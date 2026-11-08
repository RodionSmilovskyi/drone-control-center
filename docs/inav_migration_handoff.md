# Betaflight to iNAV Migration — Architecture & Implementation Handoff Guide

> [!IMPORTANT]
> **Mandatory Instructions for Implementing Agents:**
> 1. **Base Branch:** This migration **MUST** be executed starting from the **`main` branch** (not `cascaded-pid` or any experimental branch). Before editing any code, verify your branch (`git checkout main && git checkout -b inav-migration`) and read [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md) and [`GEMINI.md`](file:///home/rodion/projects/drone-control-center/GEMINI.md).
> 2. **Global Constraints:**
>    - **512MB RAM / CPU Budget (RPi Zero 2 W):** Do not introduce new Python packages or unbound queues.
>    - **Shared Memory Only for Sensors/Heartbeats:** All sensor observations and service heartbeats must pass strictly through [`core.shared_memory_manager.SharedMemoryManager`](file:///home/rodion/projects/drone-control-center/core/shared_memory_manager.py#L21-L118).
>    - **Non-Blocking ZMQ Only for IPC:** Keep `zmq.CONFLATE = 1` and `zmq.NOBLOCK` on all ZMQ sockets.
> 3. **Hardware-in-the-Loop Gate:** Never assume a step works without seeing physical Raspberry Pi + SpeedyBee F405 Mini terminal output from the user. Stop and ask the user to run each verification gate on the Pi before proceeding to the next phase.
> 4. **Design Doc Sync:** Update [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md) after completing code changes.

---

## 1. Executive Summary & Architectural Shift

### Why Migrate to iNAV (Option 2)?
On the `main` branch (with Betaflight 4.5.2), the Raspberry Pi Zero 2 W reads the downward VL53L1X ToF sensor (`I2C 0x30`), forward VL53L1X ToF sensor (`I2C 0x31`), and PMW3901 Optical Flow sensor (`SPI CS 8`), integrates position in Python without IMU gyro derotation, and runs 4-axis PIDs in [`FlightController`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py) to send raw Angle-mode roll/pitch/throttle PWM commands to Betaflight.

By switching the SpeedyBee F405 Mini to **iNAV**, we offload high-frequency **Gyro-Derotated Optical Flow Fusion**, **Inertial Velocity Estimation**, **Altitude Hold (`NAV ALTHOLD`)**, and **Position Hold (`NAV POSHOLD`)** to the STM32F405 MCU:
1. **No Hardware Rewiring Required:** The sensors stay wired to the Raspberry Pi. `drone-fc` forwards raw rangefinder (`MSP2_SENSOR_RANGEFINDER`, `0x1F01`) and optical flow (`MSP2_SENSOR_OPTICALFLOW`, `0x1F02`) packets over the existing USB serial connection (`yamspy.MSPy`).
2. **Hardware Gyro Derotation:** iNAV fuses incoming MSP optical flow deltas with its onboard 1000 Hz gyroscope and accelerometer, subtracting rotational camera motion during pitch/roll maneuvers.
3. **Velocity-Based Control (`1500 PWM = Hold`):** When `NAV ALTHOLD` and `NAV POSHOLD` are active in iNAV:
   - **`[1500, 1500, 1500, 1500]` (Roll, Pitch, Throttle, Yaw)** commands iNAV to **actively brake and hold 3D position and altitude**, automatically compensating for battery voltage sag and air drafts.
   - Stick deflections away from `1500 PWM` outside the deadband command **target 3D velocities** ($v_x, v_y, v_z$ in $\text{cm/s}$).

```mermaid
graph TD
    subgraph Sensors Service ["drone-sensors.service (30Hz)"]
        VL53L1X_Down["VL53L1X Down (I2C: 0x30)"]
        VL53L1X_Front["VL53L1X Front (I2C: 0x31)"]
        PMW3901["PMW3901 Flow (SPI CS: 8)"]
        SensProc["SensorReal (Raw dx/dy + Filtered State)"]
        VL53L1X_Down --> SensProc
        VL53L1X_Front --> SensProc
        PMW3901 --> SensProc
    end

    subgraph Shared Memory ["POSIX Shared Memory"]
        SHM_SENS["drone_sensor_data (9x float64: 72 bytes)<br/>[alt, front, sx, sy, vx, vy, ts, raw_dx, raw_dy]"]
        SHM_HB["system_heartbeats (3x float64)"]
    end

    subgraph Inference Service ["drone-inference.service (100Hz)"]
        FC_Logic["FlightController / TFLite Policy<br/>(Outputs Velocity RC around 1500 PWM)"]
    end

    subgraph Flight Controller Service ["drone-fc.service (100Hz RC / 30Hz MSP2 Sensors)"]
        FCReal["FCReal (yamspy.MSPy over USB Serial)<br/>1. MSP_SET_RAW_RC (100Hz)<br/>2. MSP2_SENSOR_RANGEFINDER 0x1F01 (30Hz)<br/>3. MSP2_SENSOR_OPTICALFLOW 0x1F02 (30Hz)"]
    end

    subgraph SpeedyBee ["SpeedyBee F405 Mini (iNAV Firmware)"]
        INAV["iNAV Estimator + NAV ALTHOLD + NAV POSHOLD<br/>(1000Hz Gyro Derotation + Accel/ToF/Flow Fusion)"]
    end

    SensProc -->|Write 9 floats| SHM_SENS
    SHM_SENS -->|Read Obs [0..6]| FC_Logic
    SHM_SENS -->|Read Raw [0, 7, 8]| FCReal
    FC_Logic -->|ZMQ PUB 5556| FCReal
    FCReal -->|USB Serial /dev/ttyACM0| INAV
```

---

## 2. Phase 1: iNAV Firmware Configuration (SpeedyBee F405 Mini)

After flashing **iNAV** (target `SPEEDYBEEF405MINI`) onto the flight controller, apply the following configuration in the **iNAV Configurator CLI** before running the RPi diagnostic script:

```text
# 1. Receiver via MSP (over USB / UART)
feature RX_MSP
set receiver_type = MSP

# 2. Enable Virtual MSP Rangefinder & Optical Flow Drivers
set rangefinder_hardware = MSP
set opflow_hardware = MSP
set opflow_scale = 10.5
set align_opflow = CW0FLIP

# 3. Indoor Navigation & Velocity Limits (Gentle Confined-Space Bounds)
set nav_mc_vel_xy_max = 50        # Max 50 cm/s horizontal speed in POSHOLD
set nav_mc_vel_z_max = 40         # Max 40 cm/s climb/descent speed in ALTHOLD
set nav_mc_bank_angle = 10        # Max 10 deg bank angle in autonomous modes
set rc_deadband = 25              # 25 PWM deadband around 1500 for Roll/Pitch
set alt_hold_deadband = 40        # 40 PWM deadband around 1500 for Throttle

# 4. Mode Ranges (Matching RPi AUX1 / AUX2 convention: [roll, pitch, throttle, yaw, aux1, aux2])
#    - Disarmed: aux1=1000, aux2=1000
#    - Armed (Angle Idle): aux1=1800, aux2=1000
#    - AI (Armed + Angle + NAV ALTHOLD + NAV POSHOLD): aux1=1800, aux2=1800
# (Configure in iNAV Configurator -> Modes tab):
#   - ARM:         AUX1 [1700 .. 2100]
#   - ANGLE:       AUX1 [1700 .. 2100]
#   - NAV ALTHOLD: AUX2 [1700 .. 2100]
#   - NAV POSHOLD: AUX2 [1700 .. 2100]

save
```

> [!NOTE]
> Notice the AUX channel improvement for iNAV:
> - In **`armed`** mode (`a` key), send `aux1 = 1800, aux2 = 1000` (Arms in `ANGLE` mode with throttle at `900` idle so motors spin safely on the ground without engaging altitude/position controllers).
> - In **`ai`** mode (`x` key), send `aux1 = 1800, aux2 = 1800` (Engages `NAV ALTHOLD` + `NAV POSHOLD`).

---

## 3. Phase 2: Pre-Flight Sensor Bridge Verification (`scripts/inav_sensor_check.py`)

Before modifying any flight control code, verify that iNAV receives, parses, and activates both the VL53L1X rangefinder and PMW3901 optical flow over MSPv2.

### 3.1 Critical Protocol Gotchas in `yamspy 0.3.3` (MUST READ)

> [!CAUTION]
> **Do NOT call `receive_msg()` after `MSP2_SENSOR_RANGEFINDER` (`0x1F01`) or `MSP2_SENSOR_OPTICALFLOW` (`0x1F02`)!**
> 1. In iNAV (`src/main/fc/fc_msp.c` $\rightarrow$ `mspProcessSensorCommand`), sensor messages in the range `0x1F00..0x1FFF` return `MSP_RESULT_NO_REPLY`. iNAV **does not send a response frame** for `0x1F01` or `0x1F02`. Calling `board.receive_msg()` after sending them will block the serial port waiting for a reply that never arrives.
> 2. Additionally, `yamspy 0.3.3` has a bug in `MSPy.receive_msg()` for MSPv2 replies (`local_read(2)` instead of `local_read(1)` on CRC8). Therefore:
>    - **For sending sensor data (`0x1F01`, `0x1F02`):** Use `board.send_RAW_msg(code, data)` **only** (fire-and-forget). `send_RAW_msg` natively constructs the `$X<` MSPv2 frame and `_crc8_dvb_s2` checksum.
>    - **For reading telemetry (`MSP_STATUS_EX` `150`, `MSP_SONAR` `58`, `MSP_ALTITUDE` `109`, `MSP_DEBUG` `254`):** Use standard MSPv1 codes (`< 255`), which `yamspy` receives and parses without error.

### 3.2 Binary Payload Formats (`msp_protocol_v2_sensor_msg.h`)

1. **`MSP2_SENSOR_RANGEFINDER` (`0x1F01`) — 5 bytes (`<Bi`):**
   ```python
   # uint8_t quality (0..255; 255 = valid), int32_t distanceMm (mm; -1 if out of range)
   payload = struct.pack("<Bi", quality, distance_mm)
   board.send_RAW_msg(0x1F01, data=list(payload))
   ```
2. **`MSP2_SENSOR_OPTICALFLOW` (`0x1F02`) — 9 bytes (`<Bii`):**
   ```python
   # uint8_t quality (0..255; 255 = valid), int32_t motionX (raw dx), int32_t motionY (raw dy)
   payload = struct.pack("<Bii", quality, int(dx), int(dy))
   board.send_RAW_msg(0x1F02, data=list(payload))
   ```

### 3.3 Diagnostic Script: [`scripts/inav_sensor_check.py`](file:///home/rodion/projects/drone-control-center/scripts/inav_sensor_check.py)

If [`scripts/inav_sensor_check.py`](file:///home/rodion/projects/drone-control-center/scripts/inav_sensor_check.py) is not yet present on your branch after checking out `main`, create it with the following exact implementation:

```python
# inav_sensor_check.py
# Standalone diagnostic script to verify:
#   1. RPi Zero 2 W hardware sensors: Down VL53L1X (I2C 0x30), Front VL53L1X (I2C 0x31), PMW3901 (SPI CS 8)
#   2. Serial MSP connection to iNAV Flight Controller (SpeedyBee F405 Mini)
#   3. MSPv2 sensor injection (MSP2_SENSOR_RANGEFINDER 0x1F01 & MSP2_SENSOR_OPTICALFLOW 0x1F02)
#   4. Live confirmation that iNAV receives, decodes, and activates Rangefinder (bit 4) & Optical Flow (bit 5)

import glob
import os
import struct
import sys
import time

import adafruit_vl53l1x
import board
import digitalio
import pmw3901
from yamspy import MSPy

# --- Configuration ---
LOG_FILE = "inav_sensor_check.log"
TEST_DURATION_SEC = 20.0
BAUD_RATE = 115200

# MSPv2 Sensor Command IDs (iNAV msp_protocol_v2_sensor.h)
MSP2_SENSOR_RANGEFINDER = 0x1F01
MSP2_SENSOR_OPTICALFLOW = 0x1F02

# iNAV activeSensors bitmask flags (MSP_STATUS / MSP_STATUS_EX)
INAV_SENSOR_ACC = 1 << 0          # 1
INAV_SENSOR_BARO = 1 << 1         # 2
INAV_SENSOR_MAG = 1 << 2          # 4
INAV_SENSOR_GPS = 1 << 3          # 8
INAV_SENSOR_RANGEFINDER = 1 << 4  # 16
INAV_SENSOR_OPFLOW = 1 << 5       # 32

# GPIO / Bus Definitions
DOWN_SENSOR_SHUT_PIN = board.D17
DOWN_SENSOR_ADDRESS = 0x30
FRONT_SENSOR_SHUT_PIN = board.D27
FRONT_SENSOR_ADDRESS = 0x31
SPI_CS_PIN = 8


def log_print(msg, log_f=None):
    """Prints message to stdout and appends to log file."""
    print(msg)
    if log_f:
        log_f.write(msg + "\n")
        log_f.flush()


def initialize_sensors(log_f=None):
    """Initializes Down/Front VL53L1X I2C sensors and PMW3901 SPI optical flow sensor."""
    log_print("--- [1/3] Initializing RPi Hardware Sensors (VL53L1X + PMW3901) ---", log_f)
    sensor_down = None
    sensor_front = None
    flow = None

    try:
        i2c = board.I2C()
        xshut_down = digitalio.DigitalInOut(DOWN_SENSOR_SHUT_PIN)
        xshut_down.direction = digitalio.Direction.OUTPUT
        xshut_front = digitalio.DigitalInOut(FRONT_SENSOR_SHUT_PIN)
        xshut_front.direction = digitalio.Direction.OUTPUT

        xshut_down.value = False
        xshut_front.value = False
        time.sleep(0.1)

        # Down Sensor (0x30)
        xshut_down.value = True
        time.sleep(0.1)
        sensor_down = adafruit_vl53l1x.VL53L1X(i2c)
        sensor_down.set_address(DOWN_SENSOR_ADDRESS)
        sensor_down.start_ranging()
        log_print(f"  [OK] Down VL53L1X online at {hex(DOWN_SENSOR_ADDRESS)}", log_f)

        # Front Sensor (0x31)
        xshut_front.value = True
        time.sleep(0.1)
        sensor_front = adafruit_vl53l1x.VL53L1X(i2c)
        sensor_front.set_address(FRONT_SENSOR_ADDRESS)
        sensor_front.start_ranging()
        log_print(f"  [OK] Front VL53L1X online at {hex(FRONT_SENSOR_ADDRESS)}", log_f)
    except Exception as e:
        log_print(f"  [FAIL] I2C VL53L1X initialization failed: {e}", log_f)

    try:
        flow = pmw3901.PMW3901(spi_cs_gpio=SPI_CS_PIN)
        flow.set_rotation(0)
        chip_id, rev = flow.get_id()
        log_print(f"  [OK] PMW3901 Optical Flow online (ID={hex(chip_id)}, Rev={hex(rev)})", log_f)
    except Exception as e:
        log_print(f"  [FAIL] SPI PMW3901 initialization failed: {e}", log_f)

    return sensor_down, sensor_front, flow


def connect_inav_fc(log_f=None):
    """Connects to the flight controller over USB serial and verifies iNAV firmware."""
    log_print("\n--- [2/3] Connecting to Flight Controller via MSP ---", log_f)
    detected = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    candidate_ports = detected if detected else ["/dev/ttyACM0", "/dev/ttyACM1"]

    for port in candidate_ports:
        if not os.path.exists(port) and len(candidate_ports) > 1:
            continue
        try:
            log_print(f"  Trying {port} @ {BAUD_RATE} baud...", log_f)
            fc = MSPy(device=port, loglevel="WARNING", baudrate=BAUD_RATE)
            if fc.__enter__() == 1:
                try:
                    fc.__exit__(None, None, None)
                except Exception:
                    pass
                continue

            for msg in ["MSP_API_VERSION", "MSP_FC_VARIANT", "MSP_FC_VERSION", "MSP_STATUS_EX"]:
                if fc.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dh = fc.receive_msg()
                    fc.process_recv_data(dh)

            variant = fc.CONFIG.get("flightControllerIdentifier", "UNKNOWN")
            version = fc.CONFIG.get("flightControllerVersion", "UNKNOWN")
            api_ver = fc.CONFIG.get("apiVersion", "UNKNOWN")
            log_print(f"  [OK] Connected on {port}: Variant={variant}, Version={version}, API={api_ver}", log_f)
            if variant != "INAV":
                log_print(
                    f"  [WARNING] FC reported variant '{variant}' (expected 'INAV'). "
                    "Flash iNAV firmware and configure rangefinder_hardware=MSP, opflow_hardware=MSP.",
                    log_f,
                )
            return fc
        except Exception as e:
            log_print(f"  Connection failed on {port}: {e}", log_f)

    return None


def send_msp2_rangefinder(fc: MSPy, distance_mm: int, quality: int = 255):
    """Sends MSP2_SENSOR_RANGEFINDER (0x1F01) to iNAV (5 bytes: uint8 quality, int32 distanceMm)."""
    payload = struct.pack("<Bi", int(quality) & 0xFF, int(distance_mm))
    fc.send_RAW_msg(MSP2_SENSOR_RANGEFINDER, data=list(payload))


def send_msp2_opticalflow(fc: MSPy, motion_x: int, motion_y: int, quality: int = 255):
    """Sends MSP2_SENSOR_OPTICALFLOW (0x1F02) to iNAV (9 bytes: uint8 quality, int32 motionX, int32 motionY)."""
    payload = struct.pack("<Bii", int(quality) & 0xFF, int(motion_x), int(motion_y))
    fc.send_RAW_msg(MSP2_SENSOR_OPTICALFLOW, data=list(payload))


def poll_inav_sensor_status(fc: MSPy):
    """Queries iNAV using MSPv1 commands to verify sensor health and fused values."""
    for cmd in ["MSP_STATUS_EX", "MSP_SONAR", "MSP_ALTITUDE", "MSP_DEBUG"]:
        if fc.send_RAW_msg(MSPy.MSPCodes[cmd], data=[]):
            dh = fc.receive_msg()
            fc.process_recv_data(dh)

    active_mask = int(fc.CONFIG.get("activeSensors", 0))
    rf_active = bool(active_mask & INAV_SENSOR_RANGEFINDER)
    of_active = bool(active_mask & INAV_SENSOR_OPFLOW)
    inav_sonar_cm = fc.SENSOR_DATA.get("sonar", 0)
    inav_est_alt_cm = fc.SENSOR_DATA.get("altitude", 0.0)
    inav_debug = list(fc.SENSOR_DATA.get("debug", [0, 0, 0, 0])[:4])
    return active_mask, rf_active, of_active, inav_sonar_cm, inav_est_alt_cm, inav_debug


if __name__ == "__main__":
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    logs_dir = os.path.join(project_root, "logs")
    os.makedirs(logs_dir, exist_ok=True)
    log_path = os.path.join(logs_dir, LOG_FILE)
    log_f = open(log_path, "w")

    log_print(f"=== iNAV MSPv2 Sensor Bridge Diagnostic ({time.strftime('%Y-%m-%d %H:%M:%S')}) ===", log_f)
    fc = None
    try:
        sensor_down, sensor_front, flow = initialize_sensors(log_f)
        fc = connect_inav_fc(log_f)

        if not (sensor_down and flow and fc):
            log_print("\n[ABORT] Prerequisite hardware (Down ToF, PMW3901, or FC serial) missing.", log_f)
            sys.exit(1)

        log_print(f"\n--- [3/3] Streaming Sensors to iNAV for {TEST_DURATION_SEC:.0f}s ---", log_f)
        log_print("Tip: Lift the drone 15-50cm above a textured floor and gently move it.", log_f)
        log_print("     In iNAV CLI, 'set debug_mode = FLOW_RAW' or 'FLOW' populates DEBUG[0..3].\n", log_f)

        start_time = time.time()
        last_status_poll = 0.0
        dist_mm = 0
        cum_dx = 0
        cum_dy = 0

        ever_rf_active = False
        ever_of_active = False
        ever_sonar_positive = False

        while time.time() - start_time < TEST_DURATION_SEC:
            # 1. Read Downward VL53L1X and stream MSP2_SENSOR_RANGEFINDER (0x1F01)
            try:
                if sensor_down.data_ready:
                    raw_cm = sensor_down.distance
                    sensor_down.clear_interrupt()
                    if raw_cm is not None and raw_cm > 0:
                        dist_mm = int(round(raw_cm * 10.0))
                        send_msp2_rangefinder(fc, distance_mm=dist_mm, quality=255)
                    else:
                        send_msp2_rangefinder(fc, distance_mm=-1, quality=0)
            except Exception:
                pass

            # 2. Read PMW3901 Optical Flow and stream MSP2_SENSOR_OPTICALFLOW (0x1F02)
            try:
                motion = flow.get_motion(timeout=0.015)
                if motion is not None:
                    dx, dy = motion
                    cum_dx += dx
                    cum_dy += dy
                    send_msp2_opticalflow(fc, motion_x=dx, motion_y=dy, quality=255)
            except RuntimeError:
                send_msp2_opticalflow(fc, motion_x=0, motion_y=0, quality=255)
            except Exception:
                pass

            # 3. Poll iNAV status & sensor telemetry at 5 Hz (every 200ms)
            now = time.time()
            if now - last_status_poll >= 0.20:
                last_status_poll = now
                try:
                    mask, rf_ok, of_ok, sonar_cm, est_alt_cm, dbg = poll_inav_sensor_status(fc)
                    ever_rf_active = ever_rf_active or rf_ok
                    ever_of_active = ever_of_active or of_ok
                    ever_sonar_positive = ever_sonar_positive or (sonar_cm > 0)

                    log_print(
                        f"  Sent: ToF={dist_mm:4d}mm FlowSum=({cum_dx:5d},{cum_dy:5d}) | "
                        f"iNAV: RF_Bit={'OK' if rf_ok else 'NO'} OF_Bit={'OK' if of_ok else 'NO'} "
                        f"Sonar={sonar_cm:3d}cm EstAlt={est_alt_cm:5.1f}cm Dbg={dbg}",
                        log_f,
                    )
                except Exception as e:
                    log_print(f"  [WARN] Telemetry poll error: {e}", log_f)

            time.sleep(0.01)

        # Final Summary Verdict
        log_print("\n=== Diagnostic Verdict ===", log_f)
        variant_ok = fc.CONFIG.get("flightControllerIdentifier") == "INAV"
        log_print(f"  1. Firmware Variant == INAV : {'PASS' if variant_ok else 'FAIL'} ({fc.CONFIG.get('flightControllerIdentifier')})", log_f)
        log_print(f"  2. iNAV Rangefinder Active  : {'PASS' if ever_rf_active else 'FAIL'} (activeSensors bit 4)", log_f)
        log_print(f"  3. iNAV Optical Flow Active : {'PASS' if ever_of_active else 'FAIL'} (activeSensors bit 5)", log_f)
        log_print(f"  4. iNAV Sonar Reading > 0cm : {'PASS' if ever_sonar_positive else 'FAIL'}", log_f)

        if variant_ok and ever_rf_active and ever_of_active and ever_sonar_positive:
            log_print("\n[SUCCESS] iNAV is receiving and decoding both Rangefinder and Optical Flow over MSPv2!", log_f)
        else:
            log_print(
                "\n[ACTION NEEDED] One or more checks failed. Verify iNAV CLI settings:\n"
                "  set rangefinder_hardware = MSP\n"
                "  set opflow_hardware = MSP\n"
                "  save",
                log_f,
            )
    finally:
        if fc is not None:
            try:
                fc.__exit__(None, None, None)
            except Exception:
                pass
        log_print(f"Log saved to: {log_path}", log_f)
        log_f.close()
```

### 3.4 Running the Sensor Check & Validating Orientation (`align_opflow`)
1. Stop background systemd services on the Pi so serial/I2C/SPI are free:
   ```bash
   sudo systemctl stop drone-fc drone-inference drone-sensors
   python3 scripts/inav_sensor_check.py
   ```
2. **Expected Output:**
   - `RF_Bit=OK` and `OF_Bit=OK`
   - `Sonar=XXcm` matches `ToF=XXXmm / 10` within `1 cm`.
3. **Verifying Optical Flow Axis Alignment & Gyro Derotation (Props OFF):**
   - In iNAV CLI, set `set debug_mode = FLOW` and `save`.
   - Re-run `python3 scripts/inav_sensor_check.py` while holding the drone ~30 cm above the floor:
     - **Roll Tilt Test:** Tilt the drone Left/Right in place (no translation) $\rightarrow$ `Dbg[0]` (flow rate X) and `Dbg[2]` (body rate X) must have the **same sign and magnitude** so their difference is near `0`.
     - **Pitch Tilt Test:** Tilt the drone Nose-Down/Nose-Up in place $\rightarrow$ `Dbg[1]` (flow rate Y) and `Dbg[3]` (body rate Y) must have the **same sign and magnitude**.
     - If X or Y is inverted or swapped, adjust `set align_opflow = ...` (`CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`, `CW90FLIP`, `CW180FLIP`, `CW270FLIP`) in iNAV CLI until both axes cancel during pure in-place rotation.

---

## 4. Phase 3: Step-by-Step Codebase Changes (From `main` Branch)

### Step 3.1: Expand Shared Memory Segment `drone_sensor_data` (`7` $\rightarrow$ `9` floats)

To pass raw PMW3901 per-frame pixel deltas (`raw_dx`, `raw_dy`) from `drone-sensors` to `drone-fc` without breaking any existing consumer of `obs[0..6]`, append two `float64` slots at the end of `drone_sensor_data`:

| Index | Field Name | Units / Range | Consumer(s) |
| :---: | :--- | :--- | :--- |
| `0` | `altitude` | meters `[0.0, 3.0]` | `drone-inference`, `drone-fc` (converted to mm for `0x1F01`), `dashboard.py` |
| `1` | `front_distance` | meters `[0.0, 3.0]` | `drone-inference` (obstacle avoidance / NN), `dashboard.py` |
| `2` | `shift_x` | meters `[-1.0, 1.0]` | `drone-inference`, `dashboard.py` |
| `3` | `shift_y` | meters `[-1.0, 1.0]` | `drone-inference`, `dashboard.py` |
| `4` | `vel_x` | normalized `[-1.0, 1.0]` | `drone-inference`, `dashboard.py` |
| `5` | `vel_y` | normalized `[-1.0, 1.0]` | `drone-inference`, `dashboard.py` |
| `6` | `timestamp` | epoch seconds | `drone-inference`, `drone-fc`, `dashboard.py` |
| `7` | `raw_flow_dx` | raw integer pixel delta X | `drone-fc` (for `MSP2_SENSOR_OPTICALFLOW` `0x1F02`) |
| `8` | `raw_flow_dy` | raw integer pixel delta Y | `drone-fc` (for `MSP2_SENSOR_OPTICALFLOW` `0x1F02`) |

#### 1. Edit [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py):
- In `SensorReal.__init__`, add:
  ```python
  self.raw_dx = 0.0
  self.raw_dy = 0.0
  ```
- In `SensorReal._poll_flow_loop`, inside `if motion is not None and current_time > warmup_end:`, store `dx, dy`:
  ```python
  with self.lock:
      self.raw_dx = float(dx)
      self.raw_dy = float(dy)
  ```
  And in the `except (RuntimeError, Exception):` block, reset `self.raw_dx = 0.0` and `self.raw_dy = 0.0`.
- In `SensorReal.read()`, return 8 values (the original 6 + `raw_dx`, `raw_dy`):
  ```python
  def read(self):
      with self.lock:
          dx, dy = self.raw_dx, self.raw_dy
          # Reset per-read deltas so duplicate reads between SPI frames send 0 delta
          self.raw_dx = 0.0
          self.raw_dy = 0.0
          return [
              self.altitude,
              self.front_dist,
              self.shift_y,
              -self.shift_x,
              self.vel_y_norm,
              -self.vel_x_norm,
              dx,
              dy,
          ]
  ```

#### 2. Edit [`services/drone-sensors/sensor_mock.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_mock.py):
- Update `SensorMock.read()` to return 8 floats (append `0.0, 0.0` for `raw_dx, raw_dy`).

#### 3. Edit [`services/drone-sensors/sensor.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor.py):
- Change `shm_size = 9 * 8` (72 bytes).
- Pack all 9 floats into `shm_data`:
  ```python
  # SHM structure: [alt, front_dist, sx, sy, vx, vy, heartbeat, raw_dx, raw_dy] (9 floats)
  shm_data = [
      raw_data[0],  # alt
      raw_data[1],  # front_dist
      raw_data[2],  # sx
      raw_data[3],  # sy
      raw_data[4],  # vx
      raw_data[5],  # vy
      time.time(),  # timestamp / heartbeat (index 6)
      raw_data[6] if len(raw_data) > 6 else 0.0,  # raw_dx (index 7)
      raw_data[7] if len(raw_data) > 7 else 0.0,  # raw_dy (index 8)
  ]
  ```

#### 4. Update `shm_size` in Consumers:
- In [`services/drone-inference/inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py): update `shm_size = 9 * 8`, `obs = np.zeros(9, dtype=np.float64)`, and `shm_mgr.read_array(np.float64, (9,))`.
- In [`dashboard.py`](file:///home/rodion/projects/drone-control-center/dashboard.py): update `shm_size = 9 * 8`, `sensor_data = np.zeros(9, dtype=np.float64)`, and `shm_mgr.read_array(np.float64, (9,))`.

---

### Step 3.2: Add MSPv2 Sensor Forwarding to `drone-fc`

#### 1. Edit [`services/drone-fc/fc_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-fc/fc_real.py):
Add `import struct` at the top of [`fc_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-fc/fc_real.py) and add `send_sensors(self, altitude_m: float, raw_dx: float, raw_dy: float)` to `FCReal`:

```python
MSP2_SENSOR_RANGEFINDER = 0x1F01
MSP2_SENSOR_OPTICALFLOW = 0x1F02
```

```python
    def send_sensors(self, altitude_m: float, raw_dx: float, raw_dy: float):
        """
        Forwards rangefinder altitude and optical flow pixel deltas to iNAV via MSPv2.
        IMPORTANT: Do NOT call receive_msg() after send_RAW_msg() for 0x1F01/0x1F02
        because iNAV returns MSP_RESULT_NO_REPLY for sensor injection frames.
        """
        if not self.board:
            return
        try:
            # 1. Rangefinder (0x1F01): <Bi (uint8 quality, int32 distance_mm)
            if altitude_m > 0.005:
                dist_mm = int(round(altitude_m * 1000.0))
                rf_payload = struct.pack("<Bi", 255, dist_mm)
            else:
                rf_payload = struct.pack("<Bi", 0, -1)
            self.board.send_RAW_msg(MSP2_SENSOR_RANGEFINDER, data=list(rf_payload))

            # 2. Optical Flow (0x1F02): <Bii (uint8 quality, int32 motionX, int32 motionY)
            of_payload = struct.pack("<Bii", 255, int(round(raw_dx)), int(round(raw_dy)))
            self.board.send_RAW_msg(MSP2_SENSOR_OPTICALFLOW, data=list(of_payload))
        except Exception as e:
            self.logger.error(f"send_sensors error: {e}")
```

#### 2. Edit [`services/drone-fc/fc_mock.py`](file:///home/rodion/projects/drone-control-center/services/drone-fc/fc_mock.py):
Add a no-op `send_sensors(self, altitude_m: float, raw_dx: float, raw_dy: float)` method so `DRONE_ENV=WSL` mock mode stays interface-compatible.

#### 3. Edit [`services/drone-fc/fc.py`](file:///home/rodion/projects/drone-control-center/services/drone-fc/fc.py):
In [`fc.py`](file:///home/rodion/projects/drone-control-center/services/drone-fc/fc.py), attach to `drone_sensor_data` (`9 * 8` bytes) and call `provider.send_sensors(...)` whenever a new sensor frame timestamp (`obs[6]`) arrives (or at least every `50ms` / `20 Hz` so iNAV's sensor timeout watchdog stays healthy):

```python
    sensor_shm_name = "drone_sensor_data"
    sensor_shm_size = 9 * 8
    sensor_shm_mgr = None
    last_sensor_ts = 0.0
    last_sensor_send_time = 0.0
```
Inside the `while True:` loop of [`fc.py`](file:///home/rodion/projects/drone-control-center/services/drone-fc/fc.py):
```python
            # 2b. Forward sensor data from SHM to iNAV when updated (or at 20Hz keepalive)
            if sensor_shm_mgr is None:
                try:
                    sensor_shm_mgr = SharedMemoryManager(sensor_shm_name, sensor_shm_size, create=False)
                except Exception:
                    sensor_shm_mgr = None

            if sensor_shm_mgr and hasattr(provider, "send_sensors"):
                try:
                    obs = sensor_shm_mgr.read_array(np.float64, (9,))
                    now = time.time()
                    sensor_ts = obs[6]
                    if (sensor_ts > 0 and sensor_ts != last_sensor_ts) or (now - last_sensor_send_time >= 0.05):
                        is_new_frame = (sensor_ts != last_sensor_ts)
                        last_sensor_ts = sensor_ts
                        last_sensor_send_time = now
                        dx = obs[7] if is_new_frame else 0.0
                        dy = obs[8] if is_new_frame else 0.0
                        provider.send_sensors(obs[0], dx, dy)
                except Exception:
                    sensor_shm_mgr.close()
                    sensor_shm_mgr = None
```
*(Remember to also call `sensor_shm_mgr.close()` in `shutdown_handler` and `finally`!)*

---

### Step 3.3: Refactor [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py) & [`inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py)

#### 1. Update Mode AUX Channels in [`services/drone-inference/inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py):
- `handle_disarmed`: returns `[1500, 1500, 900, 1500, 1000, 1000]` (`AUX1=1000` Disarmed, `AUX2=1000` Nav Off).
- `handle_armed`: change `aux2` from `1800` to `1000` $\rightarrow$ returns `[1500, 1500, 900, 1500, 1800, 1000]` (`AUX1=1800` Armed in Angle Mode at idle throttle `900`, `AUX2=1000` so iNAV `NAV ALTHOLD`/`POSHOLD` do not fight ground idle).
- `handle_ai`: keeps `aux1 = 1800, aux2 = 1800` (`[rc_roll, rc_pitch, rc_throttle, rc_yaw, 1800, 1800]`), activating `NAV ALTHOLD` + `NAV POSHOLD`.

#### 2. Replace Betaflight Angle-Mode PIDs with iNAV Velocity-Stick Mapping in [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py):
When iNAV is in `NAV ALTHOLD` + `NAV POSHOLD`:
- **Neutral Hold (`1500 PWM`):** `rc_throttle = 1500`, `rc_roll = 1500`, `rc_pitch = 1500`, `rc_yaw = 1500` tells iNAV to hold the current 3D position and altitude.
- **Moving to a Target Shift (`desired_shift_z`, `desired_shift_x`, `desired_shift_y`):**
  - Compute normalized position error:
    - $e_z = \text{desired\_alt\_norm} - \text{current\_alt\_norm}$
    - $e_x = \text{desired\_x\_norm} - \text{current\_shift\_x\_norm}$
    - $e_y = \text{desired\_y\_norm} - \text{current\_shift\_y\_norm}$
  - Within a small position arrival tolerance (e.g., $|e_z| < 0.010$ i.e. $3\text{ cm}$, $|e_{x,y}| < 0.030$ i.e. $3\text{ cm}$), output **`1500.0 PWM`** so iNAV locks onto the position inside its stick deadband.
  - Outside the arrival tolerance, command a bounded proportional velocity stick deflection outside iNAV's deadband (`alt_hold_deadband = 40`, `rc_deadband = 25`), clamped to safe indoor velocity limits (`±120 PWM` on throttle, `±80 PWM` on roll/pitch):

```python
import numpy as np
from pid_controller import PIDController

class FlightController:
    """
    High-level shift-to-velocity RC mapper for iNAV (NAV ALTHOLD + NAV POSHOLD).
    In iNAV navigation modes:
      - 1500 PWM on Roll, Pitch, Throttle, Yaw commands 0.0 m/s (Active 3D Position & Altitude Hold).
      - Deflections outside iNAV's deadband command proportional target velocities (cm/s).
    """
    def __init__(self):
        # Proportional velocity gains (used via PIDController with Ki=0, Kd=0 for interface compatibility)
        self.throttle_pid = PIDController(Kp=1.8, Ki=0.0, Kd=0.2, integral_limit=0.2, integral_min=-0.2, d_alpha=0.5)
        self.roll_pid = PIDController(Kp=1.2, Ki=0.0, Kd=0.1, integral_limit=0.2, integral_min=-0.2, d_alpha=0.5)
        self.pitch_pid = PIDController(Kp=1.2, Ki=0.0, Kd=0.1, integral_limit=0.2, integral_min=-0.2, d_alpha=0.5)
        self.yaw_pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.1, integral_limit=0.2, integral_min=-0.2)

        # In iNAV NAV ALTHOLD, 1500 PWM is zero vertical velocity (hold altitude)
        self.hover_throttle = 1500.0
        self.min_throttle = 1350.0
        self.max_throttle = 1650.0
        self.max_climb_correction = 120.0
        self.max_descent_correction = -120.0

        # iNAV deadbands (matching CLI: alt_hold_deadband = 40, rc_deadband = 25)
        self.inav_thr_deadband = 40.0
        self.inav_rp_deadband = 25.0

        # Arrival position tolerances (normalized): 3cm vertical (0.03/3.0 = 0.01), 3cm horizontal (0.03/1.0 = 0.03)
        self.alt_arrival_tol_norm = 0.010
        self.xy_arrival_tol_norm = 0.030

        # Max velocity stick deflection on Roll/Pitch (±80 PWM beyond 1500)
        self.max_rp_correction = 80.0
        self.max_yaw_correction = 50.0
        self.xy_airborne_threshold_norm = 0.050 / 3.0

        self.ref_alt_norm = None
        self.ref_shift_x_norm = None
        self.ref_shift_y_norm = None
        self.reset()

    def reset(self):
        self.throttle_pid.reset()
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.ref_alt_norm = None
        self.ref_shift_x_norm = None
        self.ref_shift_y_norm = None

    @staticmethod
    def _apply_deadband_offset(correction: float, deadband: float, max_mag: float) -> float:
        """Maps a non-zero correction outside iNAV's RC deadband so gentle commands are not ignored."""
        if abs(correction) < 1.0:
            return 0.0
        sign = 1.0 if correction > 0.0 else -1.0
        scaled = deadband + abs(correction)
        return float(sign * min(scaled, max_mag))

    def compute_rc_commands(self, high_level_action: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        if np.ndim(current_state) == 0:
            current_alt_norm = float(current_state)
            current_shift_x_norm = 0.0
            current_shift_y_norm = 0.0
            current_yaw_norm = 0.0
        else:
            current_alt_norm = float(current_state[0])
            current_shift_x_norm = float(current_state[1]) if len(current_state) > 1 else 0.0
            current_shift_y_norm = float(current_state[2]) if len(current_state) > 2 else 0.0
            current_yaw_norm = float(current_state[3]) if len(current_state) > 3 else 0.0

        desired_shift_z = float(np.clip(high_level_action[0], -1.0, 1.0))
        desired_shift_x = float(np.clip(high_level_action[1], -1.0, 1.0))
        desired_shift_y = float(np.clip(high_level_action[2], -1.0, 1.0))
        desired_yaw_norm = float(np.clip(high_level_action[3], -1.0, 1.0))

        is_xy_airborne = current_alt_norm >= self.xy_airborne_threshold_norm

        if self.ref_alt_norm is None:
            self.ref_alt_norm = current_alt_norm
        if self.ref_shift_x_norm is None:
            self.ref_shift_x_norm = current_shift_x_norm if is_xy_airborne else 0.0
        if self.ref_shift_y_norm is None:
            self.ref_shift_y_norm = current_shift_y_norm if is_xy_airborne else 0.0

        if not is_xy_airborne:
            self.ref_shift_x_norm = 0.0
            self.ref_shift_y_norm = 0.0

        desired_alt_norm = float(np.clip(self.ref_alt_norm + desired_shift_z, 0.0, 1.0))
        desired_x_norm = float(np.clip(self.ref_shift_x_norm + desired_shift_x, -1.0, 1.0))
        desired_y_norm = float(np.clip(self.ref_shift_y_norm + desired_shift_y, -1.0, 1.0))

        self.throttle_pid.setpoint = desired_alt_norm
        self.roll_pid.setpoint = desired_x_norm
        self.pitch_pid.setpoint = desired_y_norm
        self.yaw_pid.setpoint = desired_yaw_norm

        # 1. Vertical Velocity Command (Throttle around 1500 PWM)
        alt_error = desired_alt_norm - current_alt_norm
        thr_out = self.throttle_pid.compute(current_alt_norm, dt, enable_integral=False)
        if abs(alt_error) <= self.alt_arrival_tol_norm:
            thr_corr = 0.0
        else:
            raw_thr_corr = float(np.clip(100.0 * thr_out, self.max_descent_correction, self.max_climb_correction))
            thr_corr = self._apply_deadband_offset(raw_thr_corr, self.inav_thr_deadband, self.max_climb_correction)
        rc_throttle = self.hover_throttle + thr_corr

        # 2. Horizontal Velocity Commands (Roll/Pitch around 1500 PWM)
        roll_out = self.roll_pid.compute(current_shift_x_norm, dt, enable_integral=False)
        pitch_out = self.pitch_pid.compute(current_shift_y_norm, dt, enable_integral=False)
        yaw_out = self.yaw_pid.compute(current_yaw_norm, dt, enable_integral=False)

        if is_xy_airborne:
            err_x = desired_x_norm - current_shift_x_norm
            err_y = desired_y_norm - current_shift_y_norm
            roll_corr = (
                0.0
                if abs(err_x) <= self.xy_arrival_tol_norm
                else self._apply_deadband_offset(100.0 * roll_out, self.inav_rp_deadband, self.max_rp_correction)
            )
            pitch_corr = (
                0.0
                if abs(err_y) <= self.xy_arrival_tol_norm
                else self._apply_deadband_offset(100.0 * pitch_out, self.inav_rp_deadband, self.max_rp_correction)
            )
            yaw_corr = float(np.clip(100.0 * yaw_out, -self.max_yaw_correction, self.max_yaw_correction))
        else:
            roll_corr = 0.0
            pitch_corr = 0.0
            yaw_corr = 0.0

        rc_roll = 1500.0 + roll_corr
        rc_pitch = 1500.0 + pitch_corr
        rc_yaw = 1500.0 + yaw_corr

        rc_commands = np.clip([rc_throttle, rc_roll, rc_pitch, rc_yaw], 1000, 2000)
        rc_commands[0] = np.clip(rc_commands[0], self.min_throttle, self.max_throttle)
        return rc_commands.astype(np.float32)
```

---

## 5. Phase 4: Plugging In Neural Network Inference (`TFLite`) in the Future

Once iNAV `NAV ALTHOLD` + `NAV POSHOLD` is validated, integrating a trained TFLite policy into [`services/drone-inference/inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py) requires minimal changes and respects the 512MB RAM limit on the Raspberry Pi Zero 2 W:

### 5.1 One-Time Initialization in [`inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py)
Load `tflite_runtime.interpreter.Interpreter` once before entering the `while not shutting_down:` loop (never inside the 100 Hz loop):

```python
import tflite_runtime.interpreter as tflite

MODEL_PATH = os.path.join(os.path.dirname(__file__), "../../models/master-model.tflite")
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
```

### 5.2 Two Supported Neural Network Action Modes
1. **Mode A — High-Level Waypoint/Shift Policy (Compatible with `rl-ai` 7D Observation Space):**
   - **Input Tensor (`1x7` `float32`):** `[alt_norm, front_dist_norm, shift_x_norm, shift_y_norm, vel_x_norm, vel_y_norm, desired_shift_z]`
   - **Output Tensor (`1x4` `float32` in `[-1, 1]`):** `high_level_action = [desired_shift_z, desired_shift_x, desired_shift_y, desired_yaw]`
   - Pass `high_level_action` into `fc.compute_rc_commands(high_level_action, current_state, dt)`.
2. **Mode B — Direct Velocity Policy (Bypassing `FlightController`):**
   - **Output Tensor (`1x4` `float32` in `[-1, 1]`):** `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`
   - Map directly to iNAV velocity RC sticks around `1500 PWM`:
     ```python
     # 1500 PWM = 0 m/s hold; ±80 PWM = bounded indoor velocity command
     rc_roll     = int(round(1500.0 + float(np.clip(nn_out[0], -1.0, 1.0)) * 80.0))
     rc_pitch    = int(round(1500.0 + float(np.clip(nn_out[1], -1.0, 1.0)) * 80.0))
     rc_throttle = int(round(1500.0 + float(np.clip(nn_out[2], -1.0, 1.0)) * 100.0))
     rc_yaw      = int(round(1500.0 + float(np.clip(nn_out[3], -1.0, 1.0)) * 50.0))
     ```

---

## 6. Phase 5: Mandatory Validation & Testing Gate Checklist

Implementing agents **MUST** execute these gates in order and wait for user hardware output at each gate:

- [ ] **Gate 1 — Unit Tests (WSL / Local Sandbox):**
  Update [`tests/test_inference_altitude.py`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py) and [`tests/test_dashboard_sensors.py`](file:///home/rodion/projects/drone-control-center/tests/test_dashboard_sensors.py) for the 9-float `drone_sensor_data` array and iNAV `1500 PWM` hold baseline, then run `pytest`.
- [ ] **Gate 2 — On-Device Sensor Bridge Check (Props OFF):**
  Ask the user to run `python3 scripts/inav_sensor_check.py` on the Raspberry Pi and provide `logs/inav_sensor_check.log`. Confirm `RF_Bit=OK`, `OF_Bit=OK`, `Sonar > 0cm`, and verify `align_opflow` axis signs.
- [ ] **Gate 3 — Full Service Integration Check (Props OFF):**
  Start `drone-sensors`, `drone-inference`, `drone-fc`, and `dashboard.py`. Verify all 3 heartbeats are `OK`, `armed` mode sends `[1500, 1500, 900, 1500, 1800, 1000]`, and `ai` mode sends `[..., 1800, 1800]`.
- [ ] **Gate 4 — Tethered / Confined Hover Test (Props ON):**
  Engage `armed` (`a`) $\rightarrow$ `ai` (`x`) with `desired_shift_z = +0.133` (`+0.40m`, `desired_shift_x = 0.0`, `desired_shift_y = 0.0`). Confirm climb to `~0.40m`, transition of `rc_throttle` to `1500 PWM` upon reaching target altitude, and stationary `NAV POSHOLD` lock.
