# Drone Control Center — System Design & Architecture (`DESIGN.md`)

## 1. Executive Overview

The **Drone Control Center** is a modular, ultra-lightweight flight and telemetry management system designed to run on resource-constrained embedded hardware—specifically a **Raspberry Pi Zero 2 W** (512MB RAM, Quad-core ARM Cortex-A53) paired with a **Betaflight / iNAV Flight Controller** (e.g., SpeedyBee F405 Mini).

The architecture decouples hardware I/O, control/inference loops, and user interaction across independent system services communicating via **POSIX Shared Memory** (`multiprocessing.shared_memory`) and **ZeroMQ (ZMQ)** IPC channels (see diagram in [architecture_diagram.md](file:///home/rodion/projects/drone-control-center/docs/architecture_diagram.md)).

---

## 2. Architectural Principles & Constraints

1. **Strict Resource Budgeting (RPi Zero 2 W - 512MB RAM):**
   - Memory footprint is bounded and deterministic.
   - Sockets and queues avoid unbounded growth using `zmq.CONFLATE = 1`.
   - Logging is centralized and rotation-ready via `drone_logging.py`.
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
│   └── shared_memory_manager.py     # POSIX Shared Memory wrapper with resource-tracker isolation
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
├── docs/                            # Hardware & Betaflight configuration dumps
├── tests/                           # Unit and live verification tests
│   ├── test_shared_memory_manager.py
│   ├── test_policy_logic.py
│   ├── test_policy_live.py
│   ├── test_strategic_logic.py
│   └── test_dashboard_sensors.py
├── scripts/                         # Legacy/standalone prototype scripts
│   ├── fc_interface.py / fc_interface_mock.py # Legacy MQTT FC testing utilities
│   ├── simpleUI.py / rich_ui.py     # Legacy standalone telemetry/manual control consoles
│   └── inference-example.py         # Standalone TFLite experiment script
├── logs/                            # Centralized runtime log directory (*.log)
├── calibrate_sensors.py             # Interactive CLI tool to calibrate optical flow scale factor
├── sensor_check.py                  # Standalone hardware diagnostic script for VL53L1X (down & front) & PMW3901
├── dashboard.py                     # Primary Rich TUI Dashboard for live status and control
├── drone_logging.py                 # Common logging setup
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
- **Execution Rate:** ~30 Hz (background optical flow thread at ~100 Hz).
- **Sensors:**
  - **VL53L1X Downward Time-of-Flight (I2C @ 0x30, GPIO 17 XSHUT):** Distance ranging up to 3.0m with low-pass filtering ($\alpha = 0.3$).
  - **VL53L1X Forward Time-of-Flight (I2C @ 0x31, GPIO 27 XSHUT):** Distance ranging up to 3.0m with low-pass filtering ($\alpha = 0.3$).
  - **PMW3901 Optical Flow (SPI CS @ GPIO 8):** Frame displacement measurement calibrated via `FLOW_METERS_PER_PIXEL_PER_METER = 0.001997`, low-pass filter ($\alpha = 0.2$), and deadband ($0.05$).
- **Output:** Writes observations directly to `drone_sensor_data` and updates `system_heartbeats[0]`.

#### B. `drone-inference` (`services/drone-inference/`)
- **Execution Rate:** 100 Hz.
- **Inputs:**
  - `drone_sensor_data` (Shared Memory)
  - Control mode and dynamic target altitude updates from ZMQ subscriber `tcp://127.0.0.1:5555` (JSON payload `{"mode": "...", "target_alt": float}` with fallback to plain mode string).
- **Operating Modes:**
  - `disarmed`: Sends safe disarm commands `[1500, 1500, 900, 1500, 1000, 1000]`.
  - `armed`: Sends armed idle commands `[1500, 1500, 900, 1500, 1800, 1800]`.
  - `ai`: Computes PID-controlled throttle to maintain dynamic altitude setpoint (default `0.4m`, bounded `[0.1, 2.5m]`, with hover throttle baseline = `1625`, bounded `[1341, 1800]`) alongside roll/pitch/yaw commands. Altitude is normalized via $z_{\text{norm}} = z / \text{MAX\_ALTITUDE}$ (`MAX_ALTITUDE = 3.0m`), mapped to high-level action range $[-1, 1]$ via $a_{\text{alt}} = 2 \cdot z_{\text{norm}} - 1$.
- **Output:** Publishes RC packet `[roll, pitch, throttle, yaw, aux1, aux2]` over ZMQ PUB `tcp://127.0.0.1:5556`.

#### C. `drone-fc` (`services/drone-fc/`)
- **Execution Rate:** 100 Hz.
- **Inputs:** Subscribes to RC commands via ZMQ `tcp://127.0.0.1:5556`.
- **Hardware Interface:** Communicates with Betaflight via serial (`/dev/ttyACM0` @ 115200 baud) using MultiWii Serial Protocol (MSP) via `yamspy.MSPy`.
- **Output:** Dispatches raw RC channels `send_RAW_RC` and maintains connection alive to avoid flight controller failsafe timeouts.

---

### 4.3 Control & Monitoring Dashboard (`dashboard.py`)

A terminal user interface implemented with **Rich**:
- **Startup Lifecycle:** Automatically truncates all `.log` files in `logs/` upon start so session telemetry and service logs start completely from scratch.
- **Keyboard Handling:** Non-blocking raw input thread:
  - `a`: Switch to `armed`
  - `d`: Switch to `disarmed`
  - `x`: Switch to `ai` (autonomous hover/flight)
  - `w`: Increment target altitude by `+0.05m` (clamped to max `2.5m`)
  - `s`: Decrement target altitude by `-0.05m` (clamped to min `0.1m`)
  - `q`: Graceful shutdown
- **Live Panels:**
  - **Statuses:** Real-time health monitoring of all 3 microservices based on shared memory heartbeat threshold (< 1.0s = OK).
  - **Live Sensor Data:** Altitude, X/Y drift, normalized velocity, and heartbeat latency.
  - **RC Commands:** Active pulse-width values dispatched to the flight controller.
  - **System Status:** Prominently displays current flight mode (`DISARMED`, `ARMED`, `AI`), active target altitude (`Target Alt: X.XXm`), and RC pulse summary.

---

## 5. Calibration and Diagnostic Utilities

- **`sensor_check.py`:** Independent hardware test for I2C and SPI sensors, ensuring addresses, XSHUT toggles, and SPI communication are functioning before launching background daemons.
- **`calibrate_sensors.py`:** Interactive measurement helper to compute scale factors between optical flow pixel motion and physical distance in meters.
- **`scripts/rich_ui.py` / `scripts/simpleUI.py`:** Legacy manual RC control utilities over serial MSP for standalone bench testing and telemetry reception.

---

## 6. Deployment & Systemd Integration

Each service is designed to run independently under `systemd` on Raspberry Pi OS:
- `/etc/systemd/system/drone-sensors.service`
- `/etc/systemd/system/drone-inference.service`
- `/etc/systemd/system/drone-fc.service`

Environment variables:
- `DRONE_ENV=PI` (Default on hardware)
- `DRONE_ENV=WSL` (Mock sensors and FC for software-in-the-loop development)
