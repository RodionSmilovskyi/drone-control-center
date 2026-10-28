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
│   │   ├── fc.py                    # Main FC service entrypoint (ZMQ 5556 SUB -> MSP RC)
│   │   ├── fc_real.py               # Serial MSP protocol provider (via yamspy.MSPy)
│   │   └── fc_mock.py               # WSL/Mock provider
│   ├── drone-inference/             # Autonomous & Manual Control Service
│   │   ├── drone-inference.service  # Systemd unit definition
│   │   ├── inference.py             # Control loop (SHM sensor read -> PID/AI -> ZMQ 5556 PUB)
│   │   ├── flight_controller.py     # High-level action to low-level RC mapper
│   │   └── pid_controller.py        # Derivative-on-measurement PID controller
│   └── drone-sensors/               # Hardware Sensor Ingestion Service
│       ├── drone-sensors.service    # Systemd unit definition
│       ├── sensor.py                # Main sensor service entrypoint (writes SHM)
│       ├── sensor_real.py           # VL53L1X (I2C) + PMW3901 (SPI) background polling & filtering
│       └── sensor_mock.py           # Synthetic sensor data generator for WSL
├── custom_pcbs/                     # Hardware PCB schematics & gerbers
├── printed_parts/                   # 3D printable chassis & sensor mounts (.stl, .step)
├── docs/                            # Hardware, Betaflight dumps & testing handoff guides
│   ├── architecture_diagram.md
│   └── shift_pid_testing_handoff.md # 3D shift & 4-axis PID testing guide & mandatory agent tuning log
├── tests/                           # Unit and live verification tests
│   ├── test_shared_memory_manager.py
│   ├── test_inference_altitude.py
│   └── test_dashboard_sensors.py
├── scripts/                         # Standalone diagnostic, calibration & prototype scripts
│   ├── calibrate_sensors.py         # Interactive CLI tool to calibrate optical flow scale factor
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

1. **`drone_sensor_data` (7 x `float64`, 56 bytes):**
   - `[0]`: `altitude` (meters, down-facing ToF sensor)
   - `[1]`: `front_distance` (meters, front-facing ToF sensor)
   - `[2]`: `shift_x` (accumulated X displacement in meters)
   - `[3]`: `shift_y` (accumulated Y displacement in meters)
   - `[4]`: `vel_x` (normalized X velocity in `[-1.0, 1.0]`)
   - `[5]`: `vel_y` (normalized Y velocity in `[-1.0, 1.0]`)
   - `[6]`: `timestamp` (epoch timestamp of sensor update)

2. **`system_heartbeats` (3 x `float64`, 24 bytes):**
   - `Index 0`: `drone-sensors` heartbeat timestamp
   - `Index 1`: `drone-inference` heartbeat timestamp
   - `Index 2`: `drone-fc` heartbeat timestamp

*Safety Feature:* `unregister_shm()` isolates segments from Python's default `multiprocessing.resource_tracker` so that background services crashing or restarting do not prematurely delete active segments.

---

## 4.2 Services Description

### A. `drone-sensors` (`services/drone-sensors/`)
- **Execution Rate:** ~30 Hz SHM writer with two isolated background hardware threads (`_poll_tof_loop` on I2C and `_poll_flow_loop` on SPI at ~100 Hz) so optical flow waits never block altitude updates.
- **Sensors:**
  - **VL53L1X Downward Time-of-Flight (I2C @ 0x30, GPIO 17 XSHUT):** Distance ranging up to 3.0m with low-pass filtering ($\alpha = 0.6$), polled on dedicated I2C thread `_poll_tof_loop`.
  - **VL53L1X Forward Time-of-Flight (I2C @ 0x31, GPIO 27 XSHUT):** Distance ranging up to 3.0m with low-pass filtering ($\alpha = 0.3$), polled on `_poll_tof_loop`.
  - **PMW3901 Optical Flow (SPI CS @ GPIO 8):** Frame displacement measurement (`get_motion(timeout=0.025)` on dedicated SPI thread `_poll_flow_loop`) calibrated via `FLOW_METERS_PER_PIXEL_PER_METER = 0.001997`, low-pass filter ($\alpha = 0.6$), deadband ($0.05$), smooth velocity decay on timeout, and ground cutoff below `0.045m`.
- **Output:** Writes observations directly to `drone_sensor_data` and updates `system_heartbeats[0]`.

#### B. `drone-inference` (`services/drone-inference/`)
- **Execution Rate:** 100 Hz.
- **Inputs:**
  - `drone_sensor_data` (Shared Memory)
  - Control mode and dynamic 3D shift setpoints from ZMQ subscriber `tcp://127.0.0.1:5555` (JSON payload `{"mode": "...", "shift_z": float, "shift_x": float, "shift_y": float}` with fallback to `"target_alt"` or plain mode string).
- **Operating Modes:**
  - `disarmed`: Sends safe disarm commands `[1500, 1500, 900, 1500, 1000, 1000]` and resets all PID and reference states.
  - `armed`: Sends armed idle commands `[1500, 1500, 900, 1500, 1800, 1800]` and resets all PID and reference states.
  - `ai`: Accepts normalized 3D spatial shifts `desired_shift_z`, `desired_shift_x`, and `desired_shift_y` in `[-1.0, 1.0]` and computes 4-axis PID-controlled RC commands (`[roll, pitch, throttle, yaw, 1800, 1800]`):
    - **Reference State Latching:** On the first frame when `ai` mode engages, `FlightController` latches the starting normalized altitude (`ref_alt_norm = obs[0] / MAX_ALTITUDE`, `MAX_ALTITUDE = 3.0m`) and starting horizontal shifts (`ref_shift_x_norm`, `ref_shift_y_norm` with `MAX_XY_SHIFT = 1.0m`). Target normalized altitude is $\text{desired\_alt\_norm} = \text{clip}(\text{ref\_alt\_norm} + \text{desired\_shift\_z}, 0.0, 1.0)$ (default `desired_shift_z = +0.1333`, i.e., `+0.40m`), allowing both positive climbs and negative descents relative to the starting altitude.
    - **Throttle PID (`throttle_pid`):** Derivative-on-measurement (`Kp = 5.2, Ki = 1.8, Kd = 2.2`, bidirectional integral clamped to `[-0.35, 1.2]`), paired with Betaflight `vbat_sag_compensation = 100` and hover throttle baseline = `1535` (bounded `[1341, 1800]`), smooth setpoint slew/ramping ($0.35\text{ m/s}$), ground threshold (`0.018m`), and expanded asymmetrical PID correction authority (`[-150, +165]` PWM -> `[1385, 1700]` PWM).
    - **Roll (`shift_x`) & Pitch (`shift_y`) PIDs (`roll_pid`, `pitch_pid`):** Derivative-on-measurement (`Kp = 0.90, Ki = 0.25, Kd = 0.40, integral_limit = ±0.60 -> ±15 PWM max trim, d_alpha = 0.25`) with a `180 PWM/s` (`18°/s`) command slew limiter to reject optical-flow rotational tilt spikes. Parameterized explicitly against the SpeedyBee FC's `50°` max angle limit (`500 PWM` = `50°`, `10 PWM ≈ 1°`) with a safe indoor tilt cap of `max_indoor_tilt_deg = 5.0°` (`±50 PWM`, clamping Roll/Pitch to `[1450, 1550]`). Held strictly neutral (`1500 PWM`) with zero integral windup below the `0.050m` horizontal airborne threshold (`xy_airborne_threshold_norm`) for level liftoff.
    - **Yaw PID (`yaw_pid`):** Derivative-on-measurement (`Kp = 0.5, Ki = 0.0, Kd = 0.1`), maintaining neutral `1500 PWM` when yaw error is `0.0`.
    - **Testing & Tuning Log:** Detailed testing steps and a mandatory agent tuning log are maintained in [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md). Any agent adjusting PID gains, limits, or shift logic must record its changes in that handoff document.
- **Output:** Publishes RC packet `[roll, pitch, throttle, yaw, aux1, aux2]` over ZMQ PUB `tcp://127.0.0.1:5556`.

#### C. `drone-fc` (`services/drone-fc/`)
- **Execution Rate:** 100 Hz.
- **Hardware Interface:** Communicates with Betaflight/iNAV via serial using MultiWii Serial Protocol (MSP) via `yamspy.MSPy`. Features dynamic serial port detection (`/dev/ttyACM*`, `/dev/ttyUSB*`), multi-attempt initialization with DTR settle times to survive service restarts, and safe disarm sequence upon shutdown.
- **Output:** Dispatches raw RC channels `send_RAW_RC` and maintains connection alive to avoid flight controller failsafe timeouts.

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

