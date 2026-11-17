# Drone Control Center — System Design & Architecture (`DESIGN.md`)

## 1. Executive Overview

The **Drone Control Center** is a modular, ultra-lightweight flight and telemetry management system designed to run on resource-constrained embedded hardware—specifically a **Raspberry Pi Zero 2 W** (512MB RAM, Quad-core ARM Cortex-A53) paired with a **Betaflight / iNAV Flight Controller** (e.g., SpeedyBee F405 Mini).

The architecture decouples hardware I/O, control/inference loops, and user interaction across independent system services communicating via **POSIX Shared Memory** (`multiprocessing.shared_memory`) and **ZeroMQ (ZMQ)** IPC channels (see diagram in [architecture_diagram.md](file:///home/rodion/projects/drone-control-center/docs/architecture_diagram.md)).

---

## 2. Architectural Principles & Constraints

1. **Strict Resource Budgeting (RPi Zero 2 W - 512MB RAM):**
   - Memory footprint is bounded and deterministic.
   - Sockets and queues avoid unbounded growth using `zmq.CONFLATE = 1`.
   - Logging is centralized and rotation-ready via `core/drone_logging.py`.
2. **Zero-Copy / Low-Latency State Exchange:**
   - High-throughput sensor telemetry (observations, optical flow integrations) is passed strictly through POSIX Shared Memory segments (`core.shared_memory_manager.py`).
   - Prevents JSON serialization/deserialization CPU bottlenecks.
3. **Non-Blocking Inter-Process Communication:**
   - Command streaming (modes, RC channels) runs via non-blocking ZeroMQ sockets (`ZMQ.NOBLOCK`, `ZMQ.CONFLATE`).
4. **Dual Execution Environments (`PI` vs `WSL`):**
   - Controlled via `DRONE_ENV` environment variable (`PI` for physical hardware, `WSL` for desktop/mock testing without hardware connected).

---

## 3. Directory & File Structure

```
drone-control-center/
├── core/
│   ├── shared_memory_manager.py     # POSIX Shared Memory wrapper with resource-tracker isolation
│   └── drone_logging.py             # Centralized logging setup with WatchedFileHandler
├── services/
│   ├── drone-fc/                    # Flight Controller Service
│   │   ├── drone-fc.service         # Systemd unit definition
│   │   ├── fc.py                    # Main FC service entrypoint (ZMQ 5556 SUB + SHM sensors -> MSP RC & MSPv2 sensors)
│   │   ├── fc_real.py               # Serial MSP/MSPv2 protocol provider (via yamspy.MSPy)
│   │   └── fc_mock.py               # WSL/Mock provider
│   ├── drone-inference/             # Autonomous & Manual Control Service
│   │   ├── drone-inference.service  # Systemd unit definition
│   │   ├── inference.py             # Control loop (SHM sensor read -> Velocity Mapper -> ZMQ 5556 PUB)
│   │   ├── flight_controller.py     # High-level shift-to-velocity RC mapper for iNAV (NAV ALTHOLD + NAV POSHOLD)
│   │   └── pid_controller.py        # Derivative-on-measurement PID controller
│   └── drone-sensors/               # Hardware Sensor Ingestion Service
│       ├── drone-sensors.service    # Systemd unit definition
│       ├── sensor.py                # Main sensor service entrypoint (writes SHM)
│       ├── sensor_real.py           # VL53L1X (I2C) + PMW3901 (SPI) background polling & filtering
│       └── sensor_mock.py           # Synthetic sensor data generator for WSL
├── custom_pcbs/                     # Hardware PCB schematics & gerbers
├── printed_parts/                   # 3D printable chassis & sensor mounts (.stl, .step)
├── docs/                            # Hardware, FC dumps & testing handoff guides
│   ├── architecture_diagram.md
│   ├── inav_9.1.0_diff_all.txt      # Hardware-verified INAV 9.1.0 diff all backup (SPEEDYBEEF405MINI)
│   ├── inav_migration_handoff.md    # Step-by-step handoff guide for migrating from main branch to iNAV
│   └── shift_pid_testing_handoff.md # 3D shift & 4-axis PID testing guide & mandatory agent tuning log
├── tests/                           # Unit and live verification tests
│   ├── test_shared_memory_manager.py
│   ├── test_inference_altitude.py
│   └── test_dashboard_sensors.py
├── scripts/                         # Standalone diagnostic, calibration & prototype scripts
│   ├── calibrate_sensors.py         # Interactive CLI tool to calibrate optical flow scale factor
│   ├── inav_sensor_check.py         # Standalone MSPv2 sensor bridge & iNAV telemetry verification script
│   ├── sensor_check.py              # Standalone hardware diagnostic script for VL53L1X (down & front) & PMW3901
│   ├── simpleUI.py / rich_ui.py     # Legacy standalone telemetry/manual control consoles
│   └── inference-example.py         # Standalone TFLite experiment script
├── models/                          # Neural network policies & TFLite models (*.tflite)
├── logs/                            # Centralized runtime log directory (*.log)
├── dashboard.py                     # Primary Rich TUI Dashboard for live status and control
├── requirements-pi.txt              # RPi production Python dependencies
├── requirements-wsl.txt             # Development / mock environment dependencies
├── GEMINI.md                        # Embedded flight constraints & rules
└── README.md                        # Connection & operational cheatsheet
```

---

## 4. Component Details & Data Flow

### 4.1 Shared Memory Layout (`core/shared_memory_manager.py`)

1. **`drone_sensor_data` (9 x `float64`, 72 bytes):**
   - `[0]`: `altitude` (meters, down-facing ToF sensor)
   - `[1]`: `front_distance` (meters, front-facing ToF sensor)
   - `[2]`: `shift_x` (accumulated X displacement in meters)
   - `[3]`: `shift_y` (accumulated Y displacement in meters)
   - `[4]`: `vel_x` (normalized X velocity in `[-1.0, 1.0]`)
   - `[5]`: `vel_y` (normalized Y velocity in `[-1.0, 1.0]`)
   - `[6]`: `timestamp` (epoch timestamp of sensor update)
   - `[7]`: `raw_flow_dx` (accumulated raw PMW3901 optical flow pixel delta X since last read)
   - `[8]`: `raw_flow_dy` (accumulated raw PMW3901 optical flow pixel delta Y since last read)

2. **`system_heartbeats` (3 x `float64`, 24 bytes):**
   - `Index 0`: `drone-sensors` heartbeat timestamp
   - `Index 1`: `drone-inference` heartbeat timestamp
   - `Index 2`: `drone-fc` heartbeat timestamp

*Safety Features:*
- `unregister_shm()` isolates segments from Python's default `multiprocessing.resource_tracker` so that background services crashing or restarting do not prematurely delete active segments.
- When `create=True` encounters an existing segment smaller than requested (`existing.size < size`), `SharedMemoryManager` unlinks and recreates the segment at the expanded size.

---

## 4.2 Services Description

### A. `drone-sensors` (`services/drone-sensors/`)
- **Execution Rate:** ~30 Hz SHM writer with two isolated background hardware threads (`_poll_tof_loop` on I2C and `_poll_flow_loop` on SPI at ~100 Hz) so optical flow waits never block altitude updates.
- **Sensors:**
  - **VL53L1X Downward Time-of-Flight (I2C @ 0x30, GPIO 17 XSHUT):** Distance ranging up to 3.0m with low-pass filtering ($\alpha = 0.6$), polled on dedicated I2C thread `_poll_tof_loop`.
  - **VL53L1X Forward Time-of-Flight (I2C @ 0x31, GPIO 27 XSHUT):** Distance ranging up to 3.0m with low-pass filtering ($\alpha = 0.3$), polled on `_poll_tof_loop`.
  - **PMW3901 Optical Flow (SPI CS @ GPIO 8):** Frame displacement measurement (`get_motion(timeout=0.025)` on dedicated SPI thread `_poll_flow_loop`) calibrated via `FLOW_METERS_PER_PIXEL_PER_METER = 0.001997`, low-pass filter ($\alpha = 0.35$), deadband ($0.05$), smooth velocity decay on timeout, and ground cutoff below `0.045m`.
- **Output:** Accumulates raw PMW3901 pixel deltas (`raw_dx`, `raw_dy`) under `self.lock` between 30 Hz `read()` calls (resetting atomically upon `read()`), writes all 9 `float64` values (`[alt, front_dist, sx, sy, vx, vy, timestamp, raw_dx, raw_dy]`) directly to `drone_sensor_data`, and updates `system_heartbeats[0]`.

#### B. `drone-inference` (`services/drone-inference/`)
- **Execution Rate:** 100 Hz.
- **Inputs:**
  - `drone_sensor_data` (Shared Memory, 9 x `float64`)
  - Control mode and dynamic 3D shift setpoints from ZMQ subscriber `tcp://127.0.0.1:5555` (JSON payload `{"mode": "...", "shift_z": float, "shift_x": float, "shift_y": float}` with fallback to `"target_alt"` or plain mode string).
- **Operating Modes (iNAV AUX Convention):**
  - `disarmed`: Sends safe disarm commands `[1500, 1500, 900, 1500, 1000, 1000]` (`AUX1=1000` Disarmed, `AUX2=1000` Nav Off) and resets all controller and reference states.
  - `armed`: Sends armed ground-idle commands `[1500, 1500, 900, 1500, 1800, 1000]` (`AUX1=1800` Armed + Angle mode, `AUX2=1000` Nav Off so `NAV ALTHOLD`/`POSHOLD` do not fight ground idle) and resets reference states.
  - `ai`: Accepts normalized 3D spatial shifts `desired_shift_z`, `desired_shift_x`, and `desired_shift_y` in `[-1.0, 1.0]` and outputs velocity-stick RC commands around `1500 PWM` hold (`[roll, pitch, throttle, yaw, 1800, 1800]`, engaging `NAV ALTHOLD` + `NAV POSHOLD` on iNAV):
    - **Reference State Latching:** On the first frame when `ai` mode engages, `FlightController` latches the starting normalized altitude (`ref_alt_norm = obs[0] / MAX_ALTITUDE`, `MAX_ALTITUDE = 3.0m`) and starting horizontal shifts (`ref_shift_x_norm`, `ref_shift_y_norm` with `MAX_XY_SHIFT = 1.0m`, zeroed below the `0.050m` airborne threshold `xy_airborne_threshold_norm`). Target normalized altitude is $\text{desired\_alt\_norm} = \text{clip}(\text{ref\_alt\_norm} + \text{desired\_shift\_z}, 0.0, 1.0)$ (default `desired_shift_z = +0.1333`, i.e., `+0.40m`).
    - **Vertical Velocity (`throttle_pid`):** Proportional velocity mapping (`Kp = 1.8, Ki = 0.0, Kd = 0.2, d_alpha = 0.5`) centered on `hover_throttle = 1500.0` (`0 m/s` vertical hold in `NAV ALTHOLD`). Within arrival tolerance `alt_arrival_tol_norm = 0.010` (`3 cm`), outputs `1500 PWM`; outside tolerance, offsets past iNAV's `inav_thr_deadband = 40.0` (`alt_hold_deadband`) up to `±120 PWM` (`[1380, 1620]`, hard-clamped to `[1350, 1650]`).
    - **Horizontal Velocity (`roll_pid`, `pitch_pid`):** Proportional velocity mapping (`Kp = 1.2, Ki = 0.0, Kd = 0.1, d_alpha = 0.5`) centered on `1500 PWM` (`0 m/s` hold in `NAV POSHOLD`). Held strictly at `1500 PWM` below `0.050m` altitude (`xy_airborne_threshold_norm`) and within `xy_arrival_tol_norm = 0.030` (`3 cm`); outside tolerance, offsets past `inav_rp_deadband = 25.0` (`pos_hold_deadband`) up to `max_rp_correction = ±80 PWM` (`[1420, 1580]`).
    - **Yaw (`yaw_pid`):** Proportional command (`Kp = 0.5, Ki = 0.0, Kd = 0.1`), clamped to `max_yaw_correction = ±50 PWM`, maintaining neutral `1500 PWM` when yaw error is `0.0`.
    - **Testing & Tuning Log:** Detailed testing steps and a mandatory agent tuning log are maintained in [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md). Any agent adjusting PID gains, limits, or shift logic must record its changes in that handoff document.
- **Output:** Publishes RC packet `[roll, pitch, throttle, yaw, aux1, aux2]` over ZMQ PUB `tcp://127.0.0.1:5556`.

#### C. `drone-fc` (`services/drone-fc/`)
- **Execution Rate:** 100 Hz RC loop with event-driven / 20 Hz keepalive MSPv2 sensor forwarding.
- **Hardware Interface:** Communicates with iNAV (or Betaflight) via serial using MultiWii Serial Protocol (MSP/MSPv2) via `yamspy.MSPy`. Features dynamic serial port detection (`/dev/ttyACM*`, `/dev/ttyUSB*`), multi-attempt initialization with DTR settle times to survive service restarts (using MSPv1 init queries only to avoid `yamspy 0.3.3` MSPv2 `receive_msg()` CRC8 parsing issues), and safe disarm sequence upon shutdown.
- **MSPv2 Sensor Bridge (`send_sensors`):** Reads `drone_sensor_data` (`9 x float64`, 72 bytes) from Shared Memory and forwards rangefinder and optical flow frames to iNAV whenever `obs[6]` (`timestamp`) updates or at least every `0.05s` (20 Hz keepalive; sending `0, 0` optical flow deltas on keepalive repeats):
  - `MSP2_SENSOR_RANGEFINDER` (`0x1F01`, `<Bi`): `quality = 255`, `distance_mm` when `altitude_m > 0.005m` (else `quality = 0, -1`).
  - `MSP2_SENSOR_OPTICALFLOW` (`0x1F02`, `<Bii`): `quality = 255`, rounded `raw_dx`, `raw_dy`.
  - Sent fire-and-forget via `send_RAW_msg` without `receive_msg()` because iNAV returns `MSP_RESULT_NO_REPLY` for `0x1F00..0x1FFF` sensor injection commands.
- **Output:** Dispatches raw RC channels `send_RAW_RC`, streams MSPv2 sensor frames, and maintains connection alive to avoid flight controller failsafe timeouts.
- **iNAV 9.1.0 Firmware Configuration (`SPEEDYBEEF405MINI`, backup in [`docs/inav_9.1.0_diff_all.txt`](file:///home/rodion/projects/drone-control-center/docs/inav_9.1.0_diff_all.txt)):**
  - Uses `motor_pwm_protocol = MULTISHOT` (avoiding the `TIM3`/`TIM4` `DSHOT300` DMA conflict), `motorstop_on_low = OFF`, and `throttle_idle = 15` (15% = `1150 PWM` idle, exceeding the 4-in-1 ESC's `~1130–1150 PWM` startup threshold while `armed` mode keeps `rc_throttle = 900` below `min_check` so arming succeeds).
  - Configured via CLI with `aux 0..3` (`ARM`/`ANGLE` on `AUX1`, `NAV ALTHOLD`/`NAV POSHOLD` on `AUX2`) and `logic 0 1 -1 22 0 0 0 0 0` (`LOGIC_CONDITION_OVERRIDE_ARMING_SAFETY`) to bypass `WAITING FOR GPS FIX` in GPS-less indoor optical flow flight, alongside iNAV 9.1.0 bounds (`nav_manual_speed = 50`, `nav_auto_speed = 50`, `nav_mc_manual_climb_rate = 40`, `nav_mc_auto_climb_rate = 40`, `nav_mc_bank_angle = 15`, `pos_hold_deadband = 25`, `alt_hold_deadband = 40`).

---

### 4.3 Control & Monitoring Dashboard (`dashboard.py`)

A terminal user interface implemented with **Rich**:
- **Startup Lifecycle:** Automatically truncates all `.log` files in `logs/` upon start so session telemetry and service logs start completely from scratch.
- **Keyboard Handling:** Non-blocking raw input thread:
  - `a`: Switch to `armed`
  - `d`: Switch to `disarmed`
  - `x`: Switch to `ai` (autonomous hover/flight)
  - `w`: Increment `desired_shift_z` by `+0.017` (`+0.05m / 3.0m`, clamped to `1.0`)
  - `s`: Decrement `desired_shift_z` by `-0.017` (`-0.05m / 3.0m`, clamped to `-1.0`)
  - `q`: Graceful shutdown
- **Live Panels:**
  - **Statuses:** Real-time health monitoring of all 3 microservices based on shared memory heartbeat threshold (< 1.0s = OK).
  - **Live Sensor Data:** Altitude, X/Y drift, normalized velocity, and heartbeat latency.
  - **RC Commands:** Active pulse-width values dispatched to the flight controller.
  - **System Status:** Prominently displays current flight mode (`DISARMED`, `ARMED`, `AI`), active vertical shift (`Shift Z: +X.XXX (+X.XXm)`), horizontal shift targets (`Shift X/Y: +0.00, +0.00`), and RC pulse summary.

---

## 5. Calibration and Diagnostic Utilities

- **`scripts/sensor_check.py`:** Independent hardware test for I2C and SPI sensors, ensuring addresses, XSHUT toggles, and SPI communication are functioning before launching background daemons.
- **`scripts/inav_sensor_check.py`:** Standalone MSPv2 sensor bridge diagnostic that streams VL53L1X (`MSP2_SENSOR_RANGEFINDER` `0x1F01`) and PMW3901 (`MSP2_SENSOR_OPTICALFLOW` `0x1F02`) readings to an iNAV flight controller and verifies that iNAV activates its Rangefinder (`bit 4`) and Optical Flow (`bit 5`) status flags and reports valid sonar/altitude telemetry (see [`docs/inav_migration_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/inav_migration_handoff.md)).
- **`scripts/calibrate_sensors.py`:** Interactive measurement helper to compute scale factors between optical flow pixel motion and physical distance in meters.
- **`scripts/rich_ui.py`:** Standalone manual RC control utility over serial MSP for bench testing and telemetry reception.

---

## 6. Deployment & Systemd Integration

Each service is designed to run independently under `systemd` on Raspberry Pi OS:
- `/etc/systemd/system/drone-sensors.service`
- `/etc/systemd/system/drone-inference.service`
- `/etc/systemd/system/drone-fc.service`

Environment variables:
- `DRONE_ENV=PI` (Default on hardware)
- `DRONE_ENV=WSL` (Mock sensors and FC for software-in-the-loop development)

---

## 7. Simulation Synchronization (`rl-ai` Project)

The PyBullet simulation environment in `rl-ai` is calibrated to mirror physical hardware dynamics:
- **Controllers:** Exact port of `FlightController` and `PIDController` (derivative-on-measurement, low-pass filter, 1550 PWM baseline, `[-75, +130]` asymmetric PID limits, $0.35\text{ m/s}$ slew ramping, ground unweight anti-windup at $0.020\text{m}$).
- **Physical Specifications:** 475g all-up mass, LANNRC 1505 PLUS 3750KV motors, 3-inch propellers, 4S LiPo.
- **Thrust Calibration:** Calibrated `MAX_THROTTLE = 2.11807 N` per motor ($W / (4 \times 0.550)$) so that 1550 PWM RC command produces steady hover equilibrium (TWR $\approx 1.82$).
- **Observation Space (7D):** Aligned with the dual-ToF and optical flow array `[alt, front_dist, shift_x, shift_y, vel_x, vel_y, goal_alt]`.
- **Sensor Modeling:** Low-pass filtering ($\alpha = 0.6$ on altitude, $\alpha = 0.3$ on front distance), PMW3901 optical flow ground cutoff below $0.04\text{m}$, and $0.05$ deadband.
- **Episode Duration:** 120 seconds (2 minutes battery endurance) = 1,440 HRL sub-episodes (28,800 physics steps @ 240 Hz).

