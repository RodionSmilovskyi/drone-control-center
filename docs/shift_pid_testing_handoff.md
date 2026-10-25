# 3D Shift Control & 4-Axis PID — Handoff & Testing Guide

> [!IMPORTANT]
> **Mandatory Rule for AI Agents:**
> 1. Before modifying any control parameters, PID gains, axis signs, or shift logic in [`flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py), [`inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py), or [`dashboard.py`](file:///home/rodion/projects/drone-control-center/dashboard.py), read both [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md) and this handoff document.
> 2. **Log Every Adjustment:** Whenever you adjust PID gains (`Kp`, `Ki`, `Kd`), integral limits, tilt caps (`max_indoor_tilt_deg`), axis polarities, or shift logic during testing, you **MUST** append a new entry to the **[Agent Tuning & Adjustment Log](#6-agent-tuning--adjustment-log)** at the bottom of this file, recording what was changed, why (citing telemetry/log evidence), and the on-device test outcome.
> 3. Keep [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md) updated whenever architectural or parameter changes are finalized.

---

## 1. Feature Overview

The autonomous flight control pipeline in `drone-inference` has been refactored from accepting an absolute `target_altitude` (in meters) to accepting **normalized 3D spatial shifts** in `[-1.0, 1.0]`, with all four RC channels (`Throttle`, `Roll`, `Pitch`, `Yaw`) governed by dedicated [`PIDController`](file:///home/rodion/projects/drone-control-center/services/drone-inference/pid_controller.py) instances:

- **`desired_shift_z` $\in [-1.0, 1.0]$**: Desired vertical displacement as a signed fraction of `MAX_ALTITUDE` (`3.0m`), applied relative to the reference altitude latched when `ai` mode engages.
  - Positive values command a climb (e.g., `+0.133` $\approx +0.40\text{m}$).
  - Negative values command a descent from the latched reference altitude (e.g., `-0.100` $= -0.30\text{m}$).
- **`desired_shift_x` $\in [-1.0, 1.0]$**: Desired X displacement as a signed fraction of `MAX_XY_SHIFT` (`1.0m`), controlled via `roll_pid`.
- **`desired_shift_y` $\in [-1.0, 1.0]$**: Desired Y displacement as a signed fraction of `MAX_XY_SHIFT` (`1.0m`), controlled via `pitch_pid`.
- **Yaw Control**: Managed via `yaw_pid` (with `0.0` setpoint and `0.0` measurement error $\rightarrow 1500\text{ PWM}$ neutral, since `drone_sensor_data` does not include a yaw sensor and Betaflight maintains heading hold on the FC).

### Primary Initial Test Objective
**First Test Task:** Maintain the desired vertical shift (`desired_shift_z`, default `+0.133` / `+0.40m`, adjustable via `w`/`s` keys) while holding horizontal shift setpoints at zero (`desired_shift_x = 0.0`, `desired_shift_y = 0.0`) indoors.

---

## 2. Implementation Details

### 2.1 Modified Files & Responsibilities

| File | Symbol / Component | Description |
| :--- | :--- | :--- |
| [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py) | [`FlightController`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py#L4-L129) | Implements 4-axis PID control (`throttle_pid`, `roll_pid`, `pitch_pid`, `yaw_pid`), reference state latching on AI entry, `±5°` indoor bank angle limit, and level-liftoff guard. |
| [`services/drone-inference/inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py) | [`handle_ai`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py#L41-L76), [`main`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py#L78-L259) | Normalizes `obs` from shared memory (`drone_sensor_data`), receives `shift_z`, `shift_x`, `shift_y` over ZMQ `tcp://127.0.0.1:5555`, and dispatches RC commands `[roll, pitch, throttle, yaw, 1800, 1800]`. |
| [`dashboard.py`](file:///home/rodion/projects/drone-control-center/dashboard.py) | [`publish_mode_cmd`](file:///home/rodion/projects/drone-control-center/dashboard.py#L31-L39), [`keyboard_listener`](file:///home/rodion/projects/drone-control-center/dashboard.py#L167-L208) | Publishes `{"mode": ..., "shift_z": ..., "shift_x": 0.0, "shift_y": 0.0}` and adjusts `desired_shift_z` via `w`/`s` in steps of `0.017` (`~5cm`). |
| [`tests/test_inference_altitude.py`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py) | [`TestInferenceAltitude`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py#L13-L162) | Unit test suite covering reference latching, positive/negative `shift_z`, Roll/Pitch PID directions, `±50 PWM` (`±5°`) indoor tilt cap, and ground threshold guards. |

---

### 2.2 Sensor Observation & State Normalization

`drone-inference` reads the 7-element `float64` array `obs` from POSIX Shared Memory `drone_sensor_data`:
- `obs[0]`: `altitude` in meters (`[0.0, 3.0]`) $\rightarrow$ `current_alt_norm = clip(obs[0] / 3.0, 0.0, 1.0)`
- `obs[1]`: `front_distance` in meters (`[0.0, 3.0]`)
- `obs[2]`: `shift_x` in meters (`[-1.0, 1.0]`) $\rightarrow$ `current_shift_x_norm = clip(obs[2] / 1.0, -1.0, 1.0)`
- `obs[3]`: `shift_y` in meters (`[-1.0, 1.0]`) $\rightarrow$ `current_shift_y_norm = clip(obs[3] / 1.0, -1.0, 1.0)`
- `obs[4]`: `vel_x` normalized (`[-1.0, 1.0]`)
- `obs[5]`: `vel_y` normalized (`[-1.0, 1.0]`)
- `obs[6]`: `timestamp` (epoch seconds)

> [!NOTE]
> Look at [`sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py#L220-L222): `SensorReal.read()` returns `[self.altitude, self.front_dist, self.shift_y, self.shift_x, self.vel_y_norm, self.vel_x_norm]`, which [`sensor.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor.py#L77-L86) writes into `obs[0..5]`. Also note that below `0.04m` altitude, `sensor_real.py` zeroes `shift_x` and `shift_y`.

---

### 2.3 Reference State Latching (`FlightController`)

While the drone is in `disarmed` or `armed` mode, `fc.reset()` is called on every frame, keeping:
- `self.ref_alt_norm = None`
- `self.ref_shift_x_norm = None`
- `self.ref_shift_y_norm = None`
- `self.current_setpoint_norm = None`

On the **first frame** when `current_mode == "ai"`, `compute_rc_commands` latches the starting coordinates:
```python
if self.ref_alt_norm is None:
    self.ref_alt_norm = current_alt_norm
if self.ref_shift_x_norm is None:
    self.ref_shift_x_norm = current_shift_x_norm
if self.ref_shift_y_norm is None:
    self.ref_shift_y_norm = current_shift_y_norm

desired_alt_norm = float(np.clip(self.ref_alt_norm + desired_shift_z, 0.0, 1.0))
desired_x_norm = float(np.clip(self.ref_shift_x_norm + desired_shift_x, -1.0, 1.0))
desired_y_norm = float(np.clip(self.ref_shift_y_norm + desired_shift_y, -1.0, 1.0))
```

---

### 2.4 4-Axis PID Configuration & Indoor Safety Limits

All 4 controllers use derivative-on-measurement (`PIDController`) to prevent derivative kick when setpoints change:

| Axis / Channel | Controller Attribute | `Kp` | `Ki` | `Kd` | Integral Bounds | Baseline PWM | Correction Bounds (PWM) | Effective PWM Range | Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **Throttle (Z)** | `self.throttle_pid` | `5.2` | `1.0` | `3.3` | `[-0.35, +1.20]` | `1550` | `[-75, +130]` | `[1475, 1680]` | Slew rate `0.35 m/s`; anti-windup below `0.020m` |
| **Roll (X)** | `self.roll_pid` | `0.5` | `0.05` | `0.3` | `[-0.20, +0.20]` | `1500` | `[-50, +50]` (`±5°`) | `[1450, 1550]` | Locked to `1500` below `0.020m` |
| **Pitch (Y)** | `self.pitch_pid` | `0.5` | `0.05` | `0.3` | `[-0.20, +0.20]` | `1500` | `[-50, +50]` (`±5°`) | `[1450, 1550]` | Locked to `1500` below `0.020m` |
| **Yaw** | `self.yaw_pid` | `0.5` | `0.0` | `0.1` | `[-0.20, +0.20]` | `1500` | `[-50, +50]` | `[1450, 1550]` | Setpoint & state = `0.0` $\rightarrow 1500$ |

#### Degree-to-PWM Mapping on SpeedyBee F405 Mini
- Full RC stick deflection (`±500 PWM` around `1500`, i.e., `[1000, 2000]`) maps to the flight controller's maximum angle limit (`self.fc_max_angle_deg = 50.0°`), meaning **`10 PWM ≈ 1°` of tilt**.
- `self.max_indoor_tilt_deg = 5.0°` limits `self.max_rp_correction` to:
  $$\text{max\_rp\_correction} = \frac{5.0^\circ}{50.0^\circ} \times 500\text{ PWM} = 50.0\text{ PWM}$$
- With `Kp = 0.5`, a `0.20m` (`20cm`) horizontal drift produces `error = -0.20`, `100 * Kp * error = -10 PWM` ($\approx 1^\circ$ corrective bank angle).

---

## 3. Step-by-Step Testing Guide (Raspberry Pi Zero 2 W)

### Phase 1: Automated Unit Tests
Run the unit test suite on the Raspberry Pi before starting hardware tests:
```bash
python3 -m unittest tests/test_inference_altitude.py tests/test_shared_memory_manager.py -v
```
Verify that all 9 tests in `TestInferenceAltitude` and all 3 tests in `TestSharedMemoryManager` pass (`OK`).

---

### Phase 2: Props-Off / Handheld Sensor & PID Polarity Check (Critical Before First Indoor Launch)
Before spinning propellers indoors, verify that optical flow integration (`shift_x`, `shift_y`) and Roll/Pitch PID corrections oppose drift in the correct physical direction:

1. Ensure services are running:
   ```bash
   sudo systemctl restart drone-sensors drone-inference
   ```
2. Launch `dashboard.py` (with `drone-fc` stopped or battery disconnected so motors do not spin):
   ```bash
   python3 dashboard.py
   ```
3. Lift the drone by hand to **~0.20m–0.30m** above a textured surface (remember optical flow is gated off below `0.04m`).
4. Press `x` to enter **AI** mode while holding the drone level:
   - Observe **Live Sensor Data** (`Shift X`, `Shift Y`) and **RC Commands** (`Roll`, `Pitch`, `Throttle`, `Yaw`).
   - Move the drone physically **Forward**:
     - Check which sensor value (`Shift X` or `Shift Y`) changes and its sign.
     - Verify that `Pitch` RC command moves in the direction that tilts the nose **Backward** to oppose the forward drift (in Betaflight AETR, `Pitch < 1500` pitches nose back, `Pitch > 1500` pitches nose forward).
   - Move the drone physically **Right**:
     - Verify that `Roll` RC command moves in the direction that rolls the drone **Left** to oppose the rightward drift (in Betaflight AETR, `Roll < 1500` rolls left, `Roll > 1500` rolls right).
   - **If either axis pushes in the same direction as the drift (positive feedback) or if X/Y are swapped**, invert the sign or swap the axis mapping in [`flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py) / [`inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py) BEFORE attempting a live tethered/free flight!

---

### Phase 3: Live Indoor Test — Maintain Vertical Shift (`shift_z`) with `shift_x = 0, shift_y = 0`

1. Place the drone on level ground in an open indoor area.
2. Restart all three services cleanly:
   ```bash
   sudo systemctl restart drone-sensors drone-inference drone-fc
   ```
3. Start the dashboard (this automatically truncates `logs/*.log` for a clean telemetry capture):
   ```bash
   python3 dashboard.py
   ```
4. Verify all three statuses (`Sensors`, `Inference`, `FC`) show **`Ok`** in green, and `System Status` shows:
   - `DISARMED`
   - `Shift Z: +0.133 (+0.40m)`
   - `Shift X/Y: +0.00, +0.00`
5. Execute test sequence:
   - Press `a` $\rightarrow$ switches to `ARMED` (motors spin at idle `900` / Betaflight `motor_idle`).
   - Press `x` $\rightarrow$ switches to `AI`. The drone will latch ground reference altitude (`~0.013m`) and smoothly ramp its altitude setpoint at `0.35 m/s` toward `+0.40m` while holding `shift_x = 0.0` and `shift_y = 0.0`.
   - Use `w` (`+0.017` / `+5cm`) and `s` (`-0.017` / `-5cm`) to test small positive and negative vertical shift adjustments.
   - Press `d` to immediately disarm, or `q` to exit.
6. After landing and disarming, inspect the telemetry log:
   ```bash
   tail -n 100 logs/inference.log
   ```

---

## 4. Diagnostic & Tuning Guide for Follow-up Agent Sessions

When analyzing `logs/inference.log` (format: `OBS: [alt, front, sx, sy, vx, vy, ts] | RC: [roll, pitch, throttle, yaw, aux1, aux2]`):

1. **Vertical Shift (`throttle_pid`) Tracking:**
   - Check if `obs[0]` (`alt`) converges to `ref_alt + desired_shift_z * 3.0`.
   - If descending via `s` key (reducing `desired_shift_z` or using negative `desired_shift_z` from mid-air) is too slow or overshoots, inspect `max_climb_rate_norm` (`0.35 / 3.0`) and `max_descent_correction` (`-75.0`).
2. **Horizontal Drift (`roll_pid`, `pitch_pid`) Behavior:**
   - **Oscillation / Over-correction:** Reduce `Kp` (e.g. `0.5 -> 0.3`) or increase `Kd` (`0.3 -> 0.45`), or reduce `max_indoor_tilt_deg` (`5.0 -> 3.0`).
   - **Too weak to arrest drift:** Increase `Kp` (`0.5 -> 0.8`) or `max_indoor_tilt_deg` (`5.0 -> 7.0`) incrementally (never exceed `8.0°` indoors).
   - **Optical flow pitch/roll coupling:** Tilting the drone causes apparent pixel motion on the uncompensated PMW3901 sensor. Keep `Kd` moderate and `max_indoor_tilt_deg` small (`<= 5.0°`) to avoid optical-flow-to-tilt oscillation loops.

---

## 5. Verification Gate Reminder (`GEMINI.md`)

- Never assume a code or gain change works without on-device telemetry from the physical Raspberry Pi Zero 2 W.
- After making any adjustment, halt execution and ask the user to run the test on the device and provide `logs/inference.log` output.

---

## 6. Agent Tuning & Adjustment Log

> **Instructions for Agents:** Append a new dated subsection below for every change made during testing. Include:
> - **Date / Session**
> - **Files & Parameters Modified**
> - **Reason / Telemetry Observation**
> - **Validation Status / Result**

### Entry 1 — Initial Implementation (2026-10-25)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-inference/inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py)
  - [`dashboard.py`](file:///home/rodion/projects/drone-control-center/dashboard.py)
  - [`tests/test_inference_altitude.py`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Changes Made:**
  - Refactored `handle_ai` to accept `desired_shift_z`, `desired_shift_x`, `desired_shift_y` in `[-1.0, 1.0]`.
  - Added `roll_pid` (`Kp=0.5, Ki=0.05, Kd=0.3, integral_limit=±0.2`), `pitch_pid` (`Kp=0.5, Ki=0.05, Kd=0.3, integral_limit=±0.2`), and `yaw_pid` (`Kp=0.5, Ki=0.0, Kd=0.1, integral_limit=±0.2`) alongside `throttle_pid` (`Kp=5.2, Ki=1.0, Kd=3.3`).
  - Added reference state latching (`ref_alt_norm`, `ref_shift_x_norm`, `ref_shift_y_norm`) on first AI frame after `reset()`.
  - Parameterized Roll/Pitch authority in degrees (`fc_max_angle_deg = 50.0`, `max_indoor_tilt_deg = 5.0` $\rightarrow \pm 50\text{ PWM}$ `[1450, 1550]`) with a level-liftoff guard below `0.020m`.
- **Validation Status:** Local unit tests (`tests/test_inference_altitude.py`) passed (`9/9 OK`). Pending physical Raspberry Pi Zero 2 W Phase 2 (polarity check) and Phase 3 (indoor vertical shift hover test).
