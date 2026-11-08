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
    """
    Sends MSP2_SENSOR_RANGEFINDER (0x1F01) to iNAV.
    Payload layout (mspSensorRangefinderDataMessage_t, 5 bytes packed):
      - uint8_t quality    (0..255; 255 = valid)
      - int32_t distanceMm (distance in mm; negative if out of range)
    NOTE: iNAV returns MSP_RESULT_NO_REPLY for 0x1F00..0x1FFF sensor frames.
          Do NOT call fc.receive_msg() after send_RAW_msg().
    """
    payload = struct.pack("<Bi", int(quality) & 0xFF, int(distance_mm))
    fc.send_RAW_msg(MSP2_SENSOR_RANGEFINDER, data=list(payload))


def send_msp2_opticalflow(fc: MSPy, motion_x: int, motion_y: int, quality: int = 255):
    """
    Sends MSP2_SENSOR_OPTICALFLOW (0x1F02) to iNAV.
    Payload layout (mspSensorOpflowDataMessage_t, 9 bytes packed):
      - uint8_t quality (0..255; 255 = valid)
      - int32_t motionX (raw per-frame pixel delta X)
      - int32_t motionY (raw per-frame pixel delta Y)
    NOTE: iNAV returns MSP_RESULT_NO_REPLY for 0x1F00..0x1FFF sensor frames.
          Do NOT call fc.receive_msg() after send_RAW_msg().
    """
    payload = struct.pack("<Bii", int(quality) & 0xFF, int(motion_x), int(motion_y))
    fc.send_RAW_msg(MSP2_SENSOR_OPTICALFLOW, data=list(payload))


def poll_inav_sensor_status(fc: MSPy):
    """
    Queries iNAV using MSPv1 commands (MSP_STATUS_EX, MSP_SONAR, MSP_ALTITUDE, MSP_DEBUG)
    to verify what sensor values iNAV has received and fused.
    """
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
                # No motion frame ready within timeout; send zero delta with valid quality to keep iNAV watchdog alive
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
