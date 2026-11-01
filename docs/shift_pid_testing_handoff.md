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
| **Throttle (Z)** | `self.throttle_pid` | `5.2` | `1.8` | `2.2` | `[-0.35, +1.35]` | `1535` | `[-150, +185]` | `[1385, 1720]` | Slew 0.35 m/s; integral active >= 0.018m |
| **Roll (X)** | `self.roll_controller` (`vel_pid`) | `0.72` | `1.10` | `0.16` | `[-0.15, +0.15]` | `1500` | `[-50, +50]` (`±5°`) | `[1450, 1550]` | Outer `kp_pos=0.45, max_vel=0.08`, `d_alpha=0.32`, slew `500 PWM/s` |
| **Pitch (Y)** | `self.pitch_controller` (`vel_pid`) | `0.72` | `1.10` | `0.16` | `[-0.15, +0.15]` | `1500` | `[-50, +50]` (`±5°`) | `[1450, 1550]` | Outer `kp_pos=0.45, max_vel=0.08`, `d_alpha=0.32`, slew `500 PWM/s` |
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

> [!IMPORTANT]
> **MANDATORY NEXT ARCHITECTURAL STEP — Switch to Commercial-Grade Cascaded Position $\rightarrow$ Velocity Control:**
> Do **NOT** spend further iterations tuning the current single-loop horizontal position PID (`position_error -> tilt_angle`). Because horizontal tilt controls acceleration ($\ddot{x} = g\sin\theta$, a double integrator $1/s^2$ with $-180^\circ$ phase lag) on top of uncompensated optical flow, single-loop position PIDs are inherently prone to either cross-cage velocity overshoot or wall-gluing.
>
> Instead of any further single-loop gain tuning, refactor horizontal control in [`FlightController`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py) to a **2-stage Cascaded Architecture (PX4 / ArduPilot / iNAV style)**:
> 1. **Outer Position Loop (P-Controller with Hard Velocity Clamp):**
>    - Maps position error $(x_{\text{target}} - x)$ to a desired horizontal velocity $v_{\text{target}} = \text{clip}(K_{p,\text{pos}} \cdot e_x,\ -v_{\max},\ +v_{\max})$, strictly capping indoor approach speed (e.g., $v_{\max} \approx 0.10\text{–}0.12\text{ m/s}$).
> 2. **Inner Velocity Loop (PI/PID-Controller):**
>    - Maps velocity error $(v_{\text{target}} - v_{\text{measured}})$ using optical flow velocity (`obs[4]` / `obs[5]`) to Roll/Pitch tilt angle PWM. Because `tilt_angle -> velocity` is a 1st-order plant ($1/s$, $-90^\circ$ phase lag), it damps velocity directly, eliminates cross-cage sprints, and naturally learns steady-state trim (`~1490 PWM` Roll) whenever $v_{\text{measured}} \neq v_{\text{target}}$.

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

### Entry 2 — Telemetry Flight Log Analysis & Fine Tuning (2026-10-25)
- **Files Modified:**
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 370–476):**
  1. **Pitch (Y) Axis Positive Feedback:** As the drone took off, the forward ToF distance decreased from `0.324m` down to `0.092m` (drone physically moved forward), while `obs[3]` became negative (`-0.208m`). Because `pitch_pid` computed positive error from negative Y, it commanded `Pitch > 1500` (up to `1518 PWM`), pitching the nose down/forward and reinforcing the forward movement (positive feedback) until manual disarm.
  2. **Roll (X) Axis Negative Feedback:** Lateral drift reached `+0.059m` (+5.9cm), and `roll_pid` commanded `Roll = 1495 PWM` (< 1500, rolling left), successfully arresting the drift and returning X displacement to `-0.005m` (-5mm) with zero oscillation.
  3. **Vertical Hover Equilibrium:** Drone achieved clean liftoff at `1579 PWM` and hovered steadily between `0.298m` and `0.316m` with throttle at `~1585 PWM`. However, with baseline `hover_throttle = 1550` and `Ki = 1.0`, integrating the steady-state difference from `0.30m` to the `0.41m` setpoint was too sluggish during the short 3.5s flight window.
- **Adjustments Made:**
  - **Inverted Pitch Axis Polarity in `sensor_real.py`:** Mapped longitudinal displacement to `-self.shift_x` and velocity to `-self.vel_x_norm` in `SensorReal.read()` so that physical forward motion registers as positive Y displacement. Now, forward drift (`+Y`) generates negative PID error, producing `Pitch < 1500` (nose backward) to arrest forward drift.
  - **Elevated Hover Baseline & Integral Gain:** Adjusted `hover_throttle` from `1550` to `1565` (closer to empirical `1585 PWM` hover equilibrium) and increased `Ki` from `1.0` to `1.5` so altitude converges briskly to the target vertical shift without steady-state droop.
- **Validation Status:** Unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 3 — 10-Second AI Hover Flight Analysis & Fine-Tuning (2026-10-25)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 176–459):**
  1. **Pitch Polarity Inversion Confirmed Fixed:** When the drone drifted forward at $t=2.5\text{s}$ (`Sy = +0.192m`), `pitch_pid` commanded `Pitch = 1486 PWM` (< 1500, nose backward), successfully arresting forward translation. The drone reversed away from the wall, damped the backward swing, and from $t=6.0\text{s}$ to $t=9.5\text{s}$ held steady near origin (`Sy = -0.04m` to `+0.04m`, active Pitch commands within `1496–1504 PWM`).
  2. **Roll Control Stable:** Roll stayed tightly bounded (`Sx` between `-0.064m` and `+0.12m`), with corrective `Roll` commands smoothly returning the airframe to origin.
  3. **Hover Throttle Gap to Empirical Equilibrium:** As the flight progressed from $t=5.0\text{s}$ to $t=9.5\text{s}$, the actual throttle required to maintain hover on the physical airframe was observed at `1635–1652 PWM`. With `hover_throttle = 1565`, the integral term had to climb from `0.0` to `+0.448` (`+67.1 PWM`) to support hover, keeping altitude around `0.28m–0.32m` (below the `0.41m` setpoint) while the integral accumulated.
- **Adjustments Made:**
  - **Raised Baseline Hover Throttle:** Adjusted `hover_throttle` from `1565` to `1600 PWM` to bridge the gap to empirical equilibrium (`1635–1650 PWM`). Now integral only needs to trim $\pm 35\text{ PWM}$, enabling the drone to reach the `0.40m` setpoint briskly without steady-state offset.
  - **Increased Roll and Pitch Damping (`Kd`):** Increased `Kd` from `0.3` to `0.4` on both `roll_pid` and `pitch_pid` to damp horizontal translation velocity more aggressively and reduce the initial position overshoot swing from $\pm 20\text{cm}$ down to $< 10\text{cm}$.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for on-device flight validation.

### Entry 4 — Reverting Hover Throttle Baseline & Ground Shift Isolation (2026-10-25)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 246–552):**
  1. **Ground Throttle Build-up & Liftoff Pop:** With `hover_throttle = 1600 PWM`, while sitting on the ground awaiting liftoff, setpoint slewing drove P-term to `+69 PWM`, causing ground throttle to reach `1669 PWM`. When the drone finally unweighted at $t=6.1\text{s}$, it popped up abruptly to `0.525m`. Per user instruction, baseline hover throttle must not be elevated above `1550 PWM`.
  2. **Pre-Takeoff Stale Shift Latching:** Before entering AI mode, stale shift (`Sy = -0.806m`) was present in shared memory. On frame 0, `FlightController` latched `ref_shift_y_norm = -0.806`. When optical flow zeroed on the ground, error became `-0.806`, pinning `pitch_pid` to the `1450 PWM` backward tilt limit.
- **Adjustments Made:**
  - **Reverted Hover Throttle Baseline:** Set `hover_throttle` strictly back to `1550 PWM` (effective range `[1475, 1680]`), preserving smooth ground unweighting without catapulting.
  - **Ground Shift Isolation in `FlightController`:** Whenever `not is_airborne` (on ground below `0.020m`), `ref_shift_x_norm` and `ref_shift_y_norm` are held strictly at `0.0`, preventing any pre-takeoff stale coordinates from corrupting reference setpoints.
  - **Unconditional Ground Reset in `SensorReal`:** In `SensorReal._poll_loop`, whenever `new_alt < 0.04m`, `shift_x`, `shift_y`, and velocity states are unconditionally reset to `0.0` within the lock.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 5 — Fresh Battery Cage Overshoot Analysis & Throttle Authority Fix (2026-10-25)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 212–580):**
  1. **Trapped at Ceiling by Insufficient Descent Authority:** On a fresh 4S LiPo (16.8V), the drone shot up and hit the cage lid at `0.677m`. The PID commanded maximum negative correction, but `max_descent_correction = -75.0` pinned throttle to `1475 PWM` (`1550 - 75`). On a fresh pack, 1475 PWM produces $> 1.0\text{ TWR}$, making it physically impossible for the drone to descend; it stayed glued to the cage lid until manual disarm.
  2. **Violent D-Term Spike on Descent:** When the quad dipped toward 0.10m, derivative action (`Kd = 3.3`) produced a $+150\text{ PWM}$ spike, kicking throttle to `1659 PWM` and re-launching it into the ceiling.
  3. **Ground Runaway Setpoint:** Before liftoff, the setpoint slewed to `0.409m` while the quad sat on the ground, driving P-term to $+69\text{ PWM}$ (`1619 PWM` on the ground).
- **Adjustments Made:**
  - **Lowered Hover Baseline:** Reduced `hover_throttle` from `1550` to `1480 PWM` (matching fresh pack equilibrium ~1460–1480 PWM). Integral trim (`+120 PWM` headroom) effortlessly lifts throttle for sagged packs.
  - **Expanded Descent Authority:** Increased `max_descent_correction` from `-75.0` to `-135.0` (floor: `1480 - 135 = 1345 PWM`, safely above `min_throttle = 1341 PWM`), giving the PID full authority to cut power and descend immediately if it ever exceeds the target altitude.
  - **Softened D-Term Kick:** Reduced `Kd` from `3.3` to `2.0` on `throttle_pid`, preventing violent $+150\text{ PWM}$ spikes on rapid descents.
  - **Tuned Proportional & Integral Gains:** Set `Kp = 4.5` and `Ki = 1.2` on `throttle_pid`.
  - **Added Ground Error Leash:** While on the ground (`not is_airborne`), `current_setpoint_norm` is capped to at most `8cm` ($0.0267$ norm) above `current_alt_norm`. Ground P-term is strictly capped to $\le +12\text{ PWM}$ (`ground throttle` $\le 1492\text{ PWM}$), completely eliminating ground catapulting.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 6 — Liftoff Thrust Calibration & Ground Stall Resolution (2026-10-25)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 106–230):**
  1. **Ground Stall at 1492 PWM:** With `hover_throttle = 1480 PWM` and the 8cm ground leash, throttle on the ground was capped at `1492 PWM`. Because empirical liftoff thrust on this quad requires `~1570 PWM`, the quad was 80 PWM below unweighting thrust and sat on the ground without lifting off.
- **Adjustments Made:**
  - **Calibrated Baseline to 1535 PWM:** Raised `hover_throttle` to `1535 PWM`. As the setpoint slews upward at $0.35\text{ m/s}$, throttle smoothly ramps from `1535 PWM` to the exact `~1570 PWM` liftoff threshold in just $0.6\text{s}$, producing a timely, gentle liftoff without catapulting.
  - **Removed Ground Leash:** Removed the artificial 8cm ground leash so the standard slew rate guides the liftoff ramp naturally.
  - **Preserved Expanded Descent Authority & Soft D-Term:** Kept `max_descent_correction = -150.0 PWM` (floor: `1385 PWM`) so that when the quad reaches or exceeds target altitude ($0.40\text{m}$), it can actively cut throttle to descend cleanly away from the cage ceiling, and kept $K_d = 2.2$ to avoid descent kicks.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 7 — Non-Blocking PMW3901 Optical Flow Polling to Eliminate Altitude Blackouts (2026-10-25)
- **Files Modified:**
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 115–347):**
  1. **Smooth Liftoff at 0.60s (`1535 -> 1567 PWM`):** The new `1535 PWM` baseline and $0.35\text{ m/s}$ ramp achieved a gentle, timely liftoff at $t=0.60\text{s}$ (`alt = 0.072m`), climbing smoothly to `0.164m` at $t=1.85\text{s}$.
  2. **Root Cause of Overshoot & Odd Freeze/Jump Behavior — Blocking `flow.get_motion()`:**
     - In [`sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py), `self.flow.get_motion()` was called with no `timeout` argument, defaulting to `pmw3901`'s `timeout=5.0` seconds. Whenever the drone hovered still horizontally or optical flow temporarily saw zero motion (`MOT` bit `0`), `get_motion()` blocked the entire `_poll_loop` thread—including downward VL53L1X ToF altitude polling—for seconds at a time:
       - **Lines 171–200 (`19:56:25.034` to `19:56:26.043`):** All 6 sensor readings froze identically at `alt = 0.1639m` for **1.01 seconds** (30 frames). While flying blind during this 1-second sensor blackout, `throttle_pid` kept integrating the stale `0.164m` reading and ramped throttle to `1596 PWM`, driving the drone right past the `0.415m` setpoint into the cage ceiling (`0.665m` when the sensor finally unblocked at line 201).
       - **Lines 215–233 (`19:56:26.514` to `19:56:27.153`):** Sensor loop froze again for **0.64 seconds** at `alt = 0.564m`; when it unblocked at line 240, the sudden step change (`0.564m -> 0.298m -> 0.208m`) created an artificial derivative kick to `1646 PWM`.
       - **Lines 246–346 (`19:56:27.557` to `19:56:30.888`):** Sensor loop froze for **3.37 seconds** (101 frames) at `alt = 0.1720m` until manual disarm.
- **Adjustments Made:**
  - **Non-Blocking Optical Flow Read (`timeout=0.001`):** Passed `timeout=0.001` (1ms) to `self.flow.get_motion(timeout=0.001)` in [`SensorReal._poll_loop`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py#L170-L176). When no motion bit is set within 1ms (`RuntimeError`), `motion` is treated as `(0, 0)` so optical flow and velocity low-pass filters decay smoothly at 100Hz without ever blocking the loop.
  - **Immediate ToF State Commit:** Committed `self.altitude = new_alt` and `self.front_dist = new_front` under `self.lock` immediately after reading the VL53L1X sensors so ToF updates are never delayed by SPI reads.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 8 — Dedicated I2C/SPI Polling Threads & Horizontal PID Authority Tuning (2026-10-25)
- **Files Modified:**
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 83–506):**
  1. **Vertical Overshoot Eliminated:** Altitude control was rock-solid (`alt` climbed smoothly to `0.301m`, held `0.300m–0.303m` with zero blackouts, and smoothly tracked four `s`-key step-downs from `+0.40m` down to `+0.09m`).
  2. **Why Horizontal Shift Correction Was Missing (`Roll = 1500, Pitch = 1498–1499`):**
     - **Premature 1ms Burst Timeout & Filter Decay:** With `get_motion(timeout=0.001)` and `except RuntimeError: motion = (0, 0)`, two problems occurred: (a) above `0.155m` altitude (lines 278–510), PMW3901 shutter exposure time increases so a burst frame takes `5–25ms` to set `dr & 0x80`; aborting after `1ms` caused `get_motion(timeout=0.001)` to time out on 100% of frames above `0.155m` (`sx` and `sy` stayed frozen at `-0.0017m` and `+0.0124m` for 7.6s until `alt` dropped back below `0.153m` at line 511); and (b) feeding synthetic `(0, 0)` on every timeout decayed `filtered_dx`/`filtered_dy` below `FLOW_DEADBAND = 0.05` before pixel motion could integrate (`sy` only integrated `1.2cm` while `front_dist` moved `17.4cm` at lines 188–276).
     - **Low Horizontal PID Gains (`Kp = 0.5`):** Even for a `10cm` horizontal drift inside the cage, `Kp = 0.5` only produced `5 PWM` (`0.5°` tilt), which is too weak to visibly arrest drift in a confined cage.
- **Adjustments Made:**
  - **Split I2C (ToF) and SPI (Optical Flow) Into Dedicated Threads in `SensorReal`:** Created `_poll_tof_loop` (exclusively polling `sensor_down` and `sensor_front` on I2C at 100Hz so altitude never stalls) and `_poll_flow_loop` (exclusively polling `flow.get_motion(timeout=0.05)` on SPI and only updating `filtered_dx`/`filtered_dy` on valid motion frames, restoring full optical flow sensitivity at all altitudes).
  - **Increased Roll & Pitch PID Gains:** Raised `roll_pid` and `pitch_pid` from `Kp=0.5, Ki=0.05, Kd=0.4` to `Kp=1.2, Ki=0.15, Kd=0.45` (within the `±5°` / `±50 PWM` indoor safety clamp) so a `10cm` drift commands `12 PWM` (`1.2°` tilt) plus integral trim.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 9 — Eliminating Circular/Toilet-Bowl Oscillation, Ground Windup, and Low-Altitude Stall (2026-10-27)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 120–298):**
  1. **Divergent Circular / Toilet-Bowl Oscillation in Roll & Pitch:**
     - With Entry 8's aggressive horizontal gains (`Kp = 1.2, Ki = 0.15, Kd = 0.45`), initial liftoff drift (`sx = +0.076m, sy = +0.089m`) triggered strong corrective tilts (`Roll = 1488`, `Pitch = 1486`) and integral accumulation. Because $K_d / K_p$ was only $0.375$ and `Ki = 0.15` wound up during the approach, the quad accelerated right through `(0, 0)` without braking (`Roll = 1505` at `sx = 0.0m, vx = -0.031`), overshooting to `sx = -0.217m` (`Roll = 1534`) and `sy = -0.282m` (`Pitch = 1550` hard 5° rail), and then whipping back to `sx = +0.201m` (`Roll = 1457, Pitch = 1473`). Because X and Y were ~90° out of phase, the drone spiraled in widening circles.
  2. **Optical Flow Phase Lag & Uncleared Filter State (`sensor_real.py`):**
     - `ALPHA_FLOW = 0.2` applied heavy smoothing before position integration (`shift_x += d_shift_x_m`), adding ~150ms of phase lag so `shift_x`/`shift_y` continued integrating in the old direction after the drone stopped and reversed. Additionally, when `flow.get_motion(timeout=0.05)` timed out, `filtered_dx` and `filtered_dy` were not cleared.
  3. **Premature Ground Airborne Detection (`0.020m` threshold vs `0.028m–0.043m` ground reading):**
     - On the floor in `disarmed`/`armed` modes (lines 1–120), downward ToF reads `0.028m–0.031m`, and during pre-liftoff spin-up vibration (lines 134–147) reads `0.042m–0.043m`. Because `ground_threshold_norm` was `0.020m` (and flow cutoff was `0.040m`), the controller treated the stationary/vibrating drone on the floor as airborne from frame 0, integrating ground vibration into `sx`/`sy` and tilting `Roll = 1494` before liftoff.
  4. **Low-Altitude Wallowing (`0.07m–0.14m`) & Throttle Ceiling Saturation (`1674 / 1675 PWM`):**
     - While banking up to `5°` in ground-effect turbulence on a partially depleted battery, `throttle_pid` (`Kp = 4.8, Ki = 1.2`) took 6 seconds to ramp `Throttle` from `1535` up to `1674 PWM` (saturating against the `1675 PWM` ceiling at line 296) while altitude stayed trapped at `0.07m–0.14m`.
- **Adjustments Made:**
  - **Overdamped Horizontal PIDs (`roll_pid`, `pitch_pid`):** Tuned to `Kp = 0.65, Ki = 0.02, Kd = 0.55` (`integral_limit = ±0.15`). High damping-to-proportional ratio ($K_d / K_p \approx 0.85$) brakes horizontal velocity *before* crossing `(0, 0)`, and minimal `Ki = 0.02` eliminates integral-windup toilet-bowling.
  - **Reduced Optical Flow Phase Lag & Cleared Filter on Timeout (`sensor_real.py`):** Raised `ALPHA_FLOW` from `0.2` to `0.6` (`ALPHA_VEL = 0.3`) and explicitly zeroed `filtered_dx` and `filtered_dy` on `get_motion` timeout and below the ground threshold.
  - **Raised Ground Threshold to `0.045m`:** Updated `ground_threshold_norm = 0.045 / 3.0` in `FlightController` and `GROUND_ALT_THRESHOLD = 0.045` in `SensorReal` so resting (`~0.030m`) and spin-up vibration (`~0.043m`) never trigger premature PID windup or ground tilts.
  - **Boosted Vertical Climb Adaptation & Ceiling Headroom (`throttle_pid`):** Updated `throttle_pid` to `Kp = 5.2, Ki = 1.8, Kd = 2.2` and expanded `max_climb_correction` from `140.0` to `165.0` (`1700 PWM` ceiling) so the drone climbs briskly out of ground effect even on sagged batteries.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 10 — Decoupling Throttle Integral Threshold from Horizontal Airborne Gate to Fix Liftoff Deadlock (2026-10-27)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 387–594):**
  1. **Ground Liftoff Catch-22 Deadlock at `1601 PWM` (`alt = 0.0440m < 0.045m`):**
     - When AI mode engaged at line 387 (`alt = 0.0211m`), the setpoint slewed to `0.4211m` in `1.14s` (line 422), driving proportional throttle (`1535 + 100 * 5.2 * error`) to `1601 PWM`.
     - At `1601 PWM`, the drone unweighted on its landing gear to `alt = 0.0440m` (lines 451–456) — **1mm shy of the `0.045m` threshold** set in Entry 9!
     - Because `ground_threshold_norm = 0.045 / 3.0` also gated `throttle_pid`'s integral (`enable_integral = False`), `Throttle` could never increase past `1602 PWM` without crossing `0.045m`, and the drone could not cross `0.045m` on this battery state without the integral adding `+20 PWM` above `1602 PWM`. As a result, `Throttle` stayed frozen at `1600–1603 PWM` (`Roll = 1500, Pitch = 1500`) for 5.8s until manual disarm.
- **Adjustments Made:**
  - **Decoupled Throttle Integral Threshold (`ground_threshold_norm = 0.018 / 3.0`) from Horizontal Airborne Gate (`xy_airborne_threshold_norm = 0.050 / 3.0`):**
    - `throttle_pid` now enables integral accumulation whenever `alt >= 0.018m` (active on the ground at `~0.021–0.030m`), so `Throttle` smoothly continues ramping past `1601 PWM` (`+24 PWM/s` via `Ki = 1.8`) until the quad breaks ground contact and lifts off cleanly regardless of battery state.
    - `roll_pid`, `pitch_pid`, `yaw_pid`, and horizontal reference origin isolation (`ref_shift_x_norm`, `ref_shift_y_norm`) are gated by `xy_airborne_threshold_norm = 0.050 / 3.0` (`5.0cm`), keeping `Roll` and `Pitch` strictly locked at `1500 PWM` with zero horizontal integral windup until the drone is genuinely airborne above `0.050m`.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 11 — Horizontal Trim Authority, D-Term Tilt-Spike Filtering & Roll/Pitch Slew Limiting (2026-10-27)
- **Files Modified:**
  - [`services/drone-inference/pid_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/pid_controller.py)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 123–404, 9.5s AI flight on fresh battery):**
  1. **Vertical Liftoff & Altitude Hold Resolved:** Decoupling `ground_threshold_norm` (`0.018m`) from `xy_airborne_threshold_norm` (`0.050m`) worked cleanly: the drone lifted off at $t=0.30\text{s}$ (`1545 PWM`), peaked gently at `0.512m` without touching the cage ceiling, and hovered around `0.30m–0.48m` (`Throttle` `1520–1589 PWM`).
  2. **Root Cause 1 of Poor Horizontal Stabilization — Zero Effective Integral Trim Authority (`0.30 PWM` max!):**
     - With Entry 9's `Ki = 0.02` and `integral_limit = ±0.15`, the maximum possible integral output on `roll_pid` and `pitch_pid` was $100 \times 0.02 \times 0.15 = \mathbf{0.30\text{ PWM}}$ (`0.03°`), which rounds to **`0 PWM`** when cast to integer RC commands!
     - Because the physical quadcopter has a `~10–14 PWM` (`1.0°–1.4°`) natural CG/battery/accelerometer trim offset (`Roll ≈ 1488 PWM, Pitch ≈ 1514 PWM` required for zero-drift hover), having zero integral trim forced a permanent steady-state position error of $e = \frac{13\text{ PWM}}{100 \times 0.65} = \mathbf{0.20\text{m}}$ (`20cm`). Consequently, the drone drifted out to `sx = +0.212m` (line 216), then hung stuck at `sx = -0.162m to -0.178m` for 2.2s (lines 260–320) and `sy = -0.203m` for 1.5s (lines 325–350) because `Kp = 0.65` only commanded `10–14 PWM` at `16–20cm` error.
  3. **Root Cause 2 — Optical Flow Rotational Tilt-Coupling Spikes via `d_alpha = 0.6` and Unconstrained Attitude Steps:**
     - Whenever `Roll` or `Pitch` stepped abruptly, the angular rotation of the drone caused apparent pixel motion on the downward PMW3901 camera. With `0.6 * new_derivative` and `Kd = 0.55`, 1-frame rotational artifacts triggered violent `±35 PWM` D-term kicks in the opposite direction:
       - At lines 251–255 (`sx = -0.167m -> -0.120m`), `Roll` snapped from `1547 PWM` (`+4.7°` right) to `1489 PWM` (`-1.1°` left, *away* from center!) in `0.11s`, killing the return to `(0, 0)` and stranding the drone at `sx = -0.17m`.
       - At lines 376–380 (`sy = +0.142m -> +0.117m -> +0.180m`), `Pitch` chattered `1474 -> 1511 -> 1503 -> 1472 -> 1450 PWM` across 4 frames (`0.13s`).
  4. **Root Cause 3 — 60ms SPI Timeout Freezes in `_poll_flow_loop`:**
     - `flow.get_motion(timeout=0.05)` plus `time.sleep(0.01)` stalled horizontal updates for `60–100ms` (2–3 frames at lines 216–218 and 259–261) and hard-zeroed `vel_x_norm`/`vel_y_norm` on timeout.
- **Adjustments Made:**
  - **Restored Bounded Integral Trim & Proportional Stiffness (`roll_pid`, `pitch_pid`):** Tuned to `Kp = 0.90, Ki = 0.25, Kd = 0.40, integral_limit = ±0.60`. Maximum integral trim is now $100 \times 0.25 \times 0.60 = \mathbf{\pm 15.0\text{ PWM}}$ (`±1.5°`), allowing the controller to learn and cancel battery CG / trim offsets within `~2–3s` without windup circles, while `Kp = 0.90` (`9 PWM` per `10cm`) arrests initial drift within `5–10cm` and `Kd = 0.40` ($K_d / K_p = 0.44$) damps return velocity without stalling `15cm` short of origin.
  - **Added Derivative Low-Pass Filter Parameter (`d_alpha = 0.25` for Roll/Pitch):** Added `d_alpha` to [`PIDController`](file:///home/rodion/projects/drone-control-center/services/drone-inference/pid_controller.py) (default `0.6` for `throttle_pid`, `0.25` for `roll_pid`/`pitch_pid`), attenuating single-frame optical-flow rotational spikes by `2.4x`.
  - **Added Roll/Pitch Command Slew Limiter (`max_rp_slew_rate = 180.0 PWM/s`):** Capped Roll/Pitch command rate of change in [`FlightController`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py) to `180 PWM/s` (`18°/s` $\approx 6\text{ PWM}$ per `33ms` frame), preventing `30–60 PWM` single-frame attitude snaps that corrupt the downward optical flow sensor while still reaching a `20 PWM` (`2°`) corrective tilt in `0.11s`.
  - **Reduced Optical Flow Timeout to `25ms` with Smooth Velocity Decay (`sensor_real.py`):** Lowered `flow.get_motion(timeout=0.025)` and decayed `vel_x_norm`/`vel_y_norm` by `1 - ALPHA_VEL` on timeout instead of hard-zeroing or sleeping an extra 10ms.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 12 — Eliminating Zero-Crossing Integral Overshoot & Strengthening Return Braking (2026-10-27)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 120–358, 8.0s AI flight):**
  1. **Attitude Chatter Completely Eliminated & Final Hover Dead-Still:**
     - `d_alpha` filtering and `max_rp_slew_rate` eliminated all single-frame `35–60 PWM` D-term spikes.
     - Initial climb (`t = 0.0s -> 2.6s`, lines 120–198) had zero altitude overshoot (`alt = 0.366m`), arrested initial X drift at `+0.102m` (half of the previous `+0.212m`), and held `sy` within `[-0.011m, +0.033m]`.
     - In the final `1.0s` of flight (`lines 332–358`), the drone achieved a dead-still hover (`vx ≈ 0, vy ≈ 0`, `alt = 0.333m ± 0.005m`, `front_dist = 0.100m ± 0.004m`, `sx = 0.098m ± 0.003m`, `sy = 0.110m ± 0.002m`) with steady `Roll = 1489–1490, Pitch = 1493–1494`.
  2. **Root Cause of Mid-Flight Overshoot Swing (`sx -> -0.249m` at line 245, `sy -> -0.230m` at line 284):**
     - **Integral Windup Fighting Zero-Crossing Braking:** While `sx` was positive (`t = 0.4s -> 3.4s`), `Ki = 0.25` accumulated `-6.2 PWM` of integral. As the drone accelerated back toward `sx = 0` at `-0.26 m/s` (lines 220–223), that `-6.2 PWM` integral canceled out 60% of the `+10.4 PWM` derivative braking (`Roll = 1500–1503` at `sx = 0.0m`), causing the drone to coast past `0` to `-0.249m`. Similarly, while `sy` was negative (`t = 3.8s -> 6.4s`), `pitch_pid.integral` wound up to `+9.1 PWM`, canceling braking at `sy = 0.0m` (line 312, `Pitch = 1495` at `+0.50 m/s`) and holding a residual `+3.4 PWM` offset after the drone stopped at `sy = +0.110m` (lines 333–358).
     - **Insufficient Return Braking (`Kd = 0.40, d_alpha = 0.25`):** When approaching origin at `0.30–0.45 m/s`, `Kd = 0.40` with `d_alpha = 0.25` did not reverse the tilt angle early enough (`5–8cm` before origin) to bleed off momentum before crossing `0`.
- **Adjustments Made:**
  - **Reduced Horizontal Integral (`Ki = 0.06, integral_limit = ±0.50` $\rightarrow \pm 3.0\text{ PWM}$ max):** Prevents integral windup during a `1–2s` excursion (`< 1 PWM` accumulated) from canceling out derivative braking at zero-crossing or holding a post-overshoot position offset, while still trimming up to `±3 PWM` of steady bias.
  - **Increased Proportional & Derivative Braking (`Kp = 1.00, Kd = 0.65, d_alpha = 0.40, max_rp_slew_rate = 250.0 PWM/s`):**
    - When drifting away from `0` (`5cm` position + `10 cm/s` velocity), `P` and `D` combine for `11.5 PWM` (`1.15°` tilt) to arrest drift tightly.
    - When returning toward `0` (`5cm` from origin at `20 cm/s`), `D` (`+13.0 PWM`) dominates `P` (`-5.0 PWM`), applying `+7.4 PWM` braking tilt *before* reaching `(0, 0)` so the drone stops at origin without overshooting.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 13 — Eliminating High-Gain Roll Limit-Cycle Oscillation (2026-10-27)
- **Files Modified:**
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 126–463, 11.3s AI flight):**
  1. **Pitch (`sy`) & Altitude (`alt`) Were Rock-Solid for 10 Seconds:**
     - From $t=0.0\text{s}$ to $t=9.9\text{s}$ (lines 127–422), `sy` stayed within `[-0.058m, +0.084m]`, and for 3.2 consecutive seconds (`t = 6.0s -> 9.2s`, lines 308–403) held right on origin within `[-0.015m, +0.034m]` (`front_dist` steady at `0.28m–0.34m`, `Pitch` within `1494–1504 PWM`).
  2. **Why Roll (`sx`) Entered a Limit-Cycle Oscillation (`+0.087m -> -0.241m -> +0.159m -> -0.233m -> +0.173m`):**
     - On the initial climb (`t = 0.0s -> 2.1s`, lines 127–191), only **`-10 PWM` (`Roll = 1490`, `-1.0°` tilt)** was needed to completely stop rightward drift at `sx = +0.0869m` (`+8.7cm`), and only **`+4 PWM` (`Pitch = 1504`, `+0.4°` tilt)** was needed to stop backward drift at `sy = -0.034m`.
     - However, with `Kp = 1.00, Kd = 0.65` (3x–4x higher than needed for this high-TWR airframe), as `sx` returned across `0.0m`, `P + D` drove `Roll` to **`1537 PWM` (`+3.7°`)** (line 237), **`1450 PWM` (`-5.0°` hard rail)** (line 272), **`1540 PWM` (`+4.0°`)** (line 406), and **`1450 PWM`** (line 443). Tilting `3.7°–5.0°` accelerated the quadcopter to `0.60–0.80 m/s` inside the small cage (where stopping distance $\frac{v^2}{2a} \approx 27\text{cm}$ exceeds the distance to origin), and unwinding `-50 PWM` at `250 PWM/s` took 6 frames (`0.20s`) after turnaround, sustaining the side-to-side oscillation.
- **Adjustments Made:**
  - **Returned Horizontal PIDs to Gentle, Overdamped Indoor Gains (`Kp = 0.48, Ki = 0.08, Kd = 0.35, integral_limit = ±0.80 -> ±6.4 PWM max trim, d_alpha = 0.35`):**
    - Keeps corrective tilts in the `1.0°–1.5°` (`10–15 PWM`) range that empirically arrested `sx` at `8.7cm` and `sy` at `3.4cm`, preventing the `37–50 PWM` over-tilts that launch the drone across the origin at `0.60+ m/s`.
    - High damping ratio ($K_d / K_p = 0.73$) smoothly brakes return velocity (`10–15 cm/s`) without slamming into the `±50 PWM` rails.
    - `Ki = 0.08` (`±6.4 PWM` max) gently trims out the `~4–6 PWM` hover bias without winding up during swings.
  - **Increased `max_rp_slew_rate` to `350.0 PWM/s` and Set `ALPHA_FLOW = 0.35` (`sensor_real.py`):** Eliminates slew-unwind phase lag at turnaround and smooths optical flow pixel bursts.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.

### Entry 14 — Resolving Safety-Barrier Collision & Wall-Gluing via `Kd > Kp` Return Braking and Anti-Windup Unsticking (2026-10-27)
- **Files Modified:**
  - [`services/drone-inference/pid_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/pid_controller.py)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry & Physical Observations (`logs/inference.log`, flight lines 100–453, 11.85s AI flight):**
  1. **Critical Physical Insight — "Motionless" Telemetry (`t = 6.3s -> 11.85s`) Was Actually the Drone Glued to the Right/Back Safety Barrier:**
     - From `t = 4.9s` to `t = 6.3s` (lines 248–288), the drone accelerated from `shift_x = -0.270m, shift_y = +0.142m` (`front_dist = 0.158m`) all the way across the cage (`+35.2cm` right, `-30.1cm` backward, `front_dist -> 0.542m`), reaching **`+56 cm/s` right (`vel_x = +0.112`) and `-38 cm/s` back (`vel_y = -0.076`)** at line 286 before colliding with the right/back safety barrier at `t = 6.3s` (`shift_x = +0.082m, shift_y = -0.159m`).
     - From `t = 6.3s` to `t = 11.85s` (lines 288–453), the barrier physically blocked further motion (`vel_x ≈ 0, vel_y ≈ 0`, `shift_x` frozen at `+0.092m`, `shift_y` at `-0.062m`, `front_dist` at `0.388m`), so the drone stayed **glued against the safety barrier** for 5.5 seconds until manual disarm!
  2. **Root Cause 1 — Why the Drone Accelerated to `56 cm/s` Into the Barrier (`Kd < Kp` + Trim Offset):**
     - Across all flights today, the physical airframe's neutral level Roll hover trim is **`Roll ≈ 1490 PWM` (`-10 PWM`)**. Any Roll command above `1490 PWM` physically accelerates the drone to the right (`+X`).
     - During the return from `shift_x = -0.270m` (`t = 5.0s`, line 248), because $K_d = 0.35 < K_p = 0.48$ and `roll_pid.integral` had wound up positive during the `-0.270m` excursion, `Roll` stayed above `1490 PWM` (`1514 -> 1490 PWM`) all the way from `x = -0.270m` to `x = -0.050m` (line 279, `22cm` of continuous rightward acceleration!), accelerating the drone to `48–56 cm/s` before applying a single degree of leftward braking!
  3. **Root Cause 2 — Why the Drone Stayed Glued to the Barrier (`Roll = 1494 PWM, Pitch = 1503 PWM`):**
     - Once against the right/back barrier at `shift_x = +0.092m, shift_y = -0.062m` (`vel_x = 0`), `P` was only `100 * 0.48 * (-0.092) = -4.4 PWM`, and `Ki = 0.08` spent 4 seconds just unwinding the positive integral from the earlier `-0.270m` excursion (`-0.73 PWM/s`).
     - As a result, `Roll` sat at **`1494–1495 PWM`** — which is **still `+4 to +5 PWM` tilted to the RIGHT relative to the `1490 PWM` level trim** — continuously pressing the drone into the safety barrier while `Throttle` roared at `1700 PWM`!
- **Adjustments Made:**
  - **Added Return-Velocity & Zero-Crossing Anti-Windup (`anti_windup_return=True`) in [`PIDController`](file:///home/rodion/projects/drone-control-center/services/drone-inference/pid_controller.py):**
    - **Crossed Setpoint (`integral * error < 0`):** Unwinds opposing integral **3.5x faster** (`3.5 * error * dt`) so windup from one side of the cage never pushes the drone into the opposite barrier.
    - **Moving Toward Setpoint (`error * derivative < 0` and `|v| > 8 cm/s`):** Pauses integral accumulation (gently bleeds `10%/s`) as soon as the drone is already returning toward `(0, 0)` faster than `8 cm/s`, preventing overshoot windup while preserving the learned hover trim.
    - **Drifting Away or Stalled Against Barrier (`|v| <= 8 cm/s`):** Integrates at full `Ki = 0.35` rate to rapidly overcome trim bias and wall suction.
  - **Tuned Horizontal PIDs to Velocity-Dominated Braking (`Kp = 0.75, Ki = 0.35, Kd = 0.85, integral_limit = ±0.50 -> ±17.5 PWM max trim, d_alpha = 0.35`):**
    - **`Kd > Kp` ($K_d / K_p = 1.13\text{ s}$):** Caps return approach velocity to $\le 12–15\text{ cm/s}$ by reversing into a braking tilt as soon as return speed exceeds $0.88 |x|$, preventing cross-cage sprints into the barrier.
    - **Strong Unsticking Authority (`P + I`):** If stalled against a barrier at `9.2cm` (`v = 0`), `P` (`-6.9 PWM`) + `I` (`-3.2 PWM/s` up to `-17.5 PWM`) rapidly reaches **`-24.4 PWM` (`Roll = 1476 PWM`, `-14 PWM` past level trim)** to peel the drone off the barrier within `< 1s`, then immediately brakes via `Kd = 0.85` as soon as it moves away from the wall.
  - **Expanded Climb Throttle Headroom (`max_climb_correction = 185.0 -> 1720 PWM ceiling`, `throttle_pid.integral_limit = 1.35`):**
    - Prevents `throttle_pid` from saturating at `1700 PWM` (`0.29m`) as the battery drains.
- **Validation Status:** Local unit tests passed (`12/12 OK`). Ready for Raspberry Pi verification.
- **Next Step Directive:** If single-loop horizontal stabilization in Entry 14 is still not satisfactory, **stop tuning the single-loop position PID** and immediately transition to the **commercial-grade Cascaded Position (P) $\rightarrow$ Velocity (PI/PID) architecture** documented in Section 4 above.

### Entry 15 — Implementation of Cascaded Position (P) $\rightarrow$ Velocity (PI/PID) Architecture (2026-10-28)
- **Files Modified:**
  - [`services/drone-inference/cascaded_pid.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/cascaded_pid.py) (NEW)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-inference/inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py)
  - [`dashboard.py`](file:///home/rodion/projects/drone-control-center/dashboard.py)
  - [`tests/test_inference_altitude.py`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Motivation & Theoretical Basis:**
  - Single-loop horizontal position PID (`position_error -> tilt_angle`) is inherently limited because horizontal tilt controls acceleration ($\ddot{x} = g\sin\theta$, a double integrator $1/s^2$ with $-180^\circ$ phase lag). This required brittle ad-hoc anti-windup heuristics to prevent either cross-cage sprints or wall-gluing.
  - Furthermore, future phases require the drone not just to maintain hover, but to autonomously translate to arbitrary 3D spatial shifts (e.g. $+0.1z, +0.3x, -0.1y$) and hold steady. Single-loop controllers subjected to a $+0.30\text{m}$ step command command excessive tilt and sprint into boundaries.
  - Cascaded control splits horizontal control into two robust stages (PX4 / ArduPilot / iNAV style):
    1. **Outer Loop (Position P with Hard Velocity Clamp):**
       Maps position error $(x_{\text{target}} - x)$ to target velocity $v_{\text{target}} = \text{clip}(K_{p,\text{pos}} \cdot e_x, -v_{\max}, +v_{\max})$.
       With $v_{\max} = 0.12\text{ m/s}$ and $K_{p,\text{pos}} = 1.0\text{ s}^{-1}$, the approach speed is strictly bounded, smoothly leading the drone to any waypoint shift without runaway acceleration.
    2. **Inner Loop (Velocity PI/PID):**
       Maps velocity error $(v_{\text{target}} - v_{\text{measured}})$ using optical flow velocity (`obs[4]` / `obs[5]` scaled by `MAX_VELOCITY = 5.0 m/s`) to tilt correction PWM. Because `tilt_angle -> velocity` is a 1st-order plant ($1/s$, $-90^\circ$ phase lag), it damps velocity directly, applies pre-emptive braking whenever return speed exceeds $v_{\text{target}}$, and eliminates position integral windup.
- **Architectural Simplifications & Parameters:**
  - Replaced brittle ad-hoc `anti_windup_return` heuristics with clean standard bounded velocity integration.
  - **Outer Loop:** $K_{p,\text{pos}} = 1.0\text{ s}^{-1}$, $v_{\max} = 0.12\text{ m/s}$.
  - **Inner Loop:** $K_{p,\text{vel}} = 1.5$, $K_{i,\text{vel}} = 0.8$, $K_{d,\text{vel}} = 0.05$, `integral_limit = ±0.25` ($\pm 20\text{ PWM}$ max trim), `d_alpha = 0.35`.
  - **Safety Guards Preserved:** Max indoor tilt limit `±5.0°` (`±50 PWM`), command slew rate limiter `350 PWM/s`, ground gate neutral hold (`1500 PWM`) below `0.050m`.
  - **Dashboard Controls Added:** Keyboard keys `i`/`k` (Pitch shift $\pm 5\text{cm}$) and `j`/`l` (Roll shift $\pm 5\text{cm}$) added to [`dashboard.py`](file:///home/rodion/projects/drone-control-center/dashboard.py) alongside existing `w`/`s` (Altitude $\pm 5\text{cm}$).
- **Validation Status:** All 17 automated unit tests passed (`17/17 OK`). Ready for hardware-in-the-loop validation on the physical Raspberry Pi Zero 2 W.

### Entry 16 — Resolving Cascaded Velocity Integral Windup & Double-LPF Phase Delay (2026-11-01)
- **Files Modified:**
  - [`services/drone-inference/cascaded_pid.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/cascaded_pid.py)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`tests/test_inference_altitude.py`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 304–591, 9.6s AI flight):**
  1. **What Improved:**
     - During the first `2.8s` of flight (`lines 318–391`), the cascaded controller held `shift_x` within `[-0.030m, +0.008m]` (`≤ 3cm`) and `shift_y` within `[-0.010m, +0.026m]` (`≤ 2.6cm`) while climbing smoothly to `0.306m`.
     - Wall-gluing was completely eliminated: the drone never stayed stuck against any safety barrier.
  2. **Root Cause 1 — Inner Velocity Integrator (`ki_vel = 0.8`) Acting as a Second Position Integrator & Corrupting Hover Trim During High-Speed Returns:**
     - From `t = 3.0s` to `3.8s` (lines 391–417), as `shift_x` drifted right to `+0.110m` (`velocity_error = -0.26 m/s`), `ki_vel = 0.8` slewed the inner velocity integral at `-20.8 PWM/s`, hitting the `-20.0 PWM` rail (`Roll = 1458 PWM`, `-4.2°` bank) in `< 1.0s`.
     - When the drone accelerated left and crossed `target_velocity` (`-0.10 m/s`) at line 421 (`shift_x = +0.098m`), `vel_pid.integral` was still pinned at `-20.0 PWM`, keeping `Roll` below `1500` until line 422 (`-0.15 m/s`) and launching the drone across the origin at **`-0.65 m/s`** (`obs[4] = -0.130` at line 438, `shift_x = -0.181m`).
     - Worse, during that `-0.65 m/s` leftward overshoot (`velocity_error = +0.77 m/s`), the unconstrained velocity integrator integrated the transient braking error at `+61 PWM/s`, overwriting the learned leftward Roll hover trim (`~ -10 PWM`) with **`+20.0 PWM` (the wrong sign!)**, which then launched the drone back to `shift_x = +0.135m` (line 533). The exact same integral windup cycle occurred on `Pitch` (`lines 485–575`, oscillating between `Pitch = 1550` and `1450`).
  3. **Root Cause 2 — ~140ms Double-LPF Velocity Sensing Lag in `sensor_real.py` + Weak Acceleration Damping (`kd_vel = 0.05`):**
     - `sensor_real.py` filtered raw optical flow with `ALPHA_FLOW = 0.35` ($\tau_1 \approx 61\text{ms}$) and then filtered `raw_vx`/`raw_vy` a second time with `ALPHA_VEL = 0.30` ($\tau_2 \approx 77\text{ms}$), delaying velocity feedback by `~140ms`. Combined with `kd_vel = 0.05` (which produced `< 1 PWM` of acceleration damping), the inner loop reacted too late to arrest momentum before hitting the `±50 PWM` rails.
  4. **Root Cause 3 — Outer Position Loop Approach Speed (`kp_pos = 1.0, max_velocity = 0.12 m/s`) Too Fast for Confined Cage:**
     - Commanding `10–12 cm/s` return speed inside the cage meant the inner loop did not begin braking until the drone was already moving faster than `10–12 cm/s` close to origin.
- **Adjustments Made:**
  - **Added Bounded, Return-Aware Trim Integration in [`CascadedAxisController`](file:///home/rodion/projects/drone-control-center/services/drone-inference/cascaded_pid.py):**
    - Clamped the velocity error fed into the trim integrator to `max_integral_vel_error = 0.08 m/s` with `ki_vel = 0.30` (`integral_limit = ±0.45 -> ±13.5 PWM` max trim), bounding maximum integral slew rate to `±2.4 PWM/s` so high-speed transient swings can never overwrite the learned hover trim.
    - When `position_error * velocity_error < 0` (drone is returning toward target faster than `target_velocity`), any integral that was pushing toward target unwinds `2.0x` faster, and opposing (braking) integral accumulation is frozen (`0.0`) so braking a return never winds up a reverse bounce.
    - During high-speed overshoots away from target (`|current_vel| > max_velocity`), wrong-sign integral unwinds `2.5x` faster while right-sign trim accumulates at `0.35x` rate while `P_vel` and `D_vel` handle dynamic braking.
  - **Tuned Outer & Inner Cascaded Gains (`kp_pos = 0.60, max_velocity = 0.08 m/s, kp_vel = 1.20, ki_vel = 0.30, kd_vel = 0.18, d_alpha = 0.45`):**
    - Lowered outer loop approach speed cap to `8 cm/s` (`kp_pos = 0.60`, so `10cm` error targets `6 cm/s` and `5cm` error targets `3 cm/s`), triggering pre-emptive braking much earlier on return.
    - Increased inner loop acceleration damping `kd_vel` from `0.05` to `0.18` (`3.6x` increase) with `d_alpha = 0.45` to provide phase-lead braking against attitude and sensor delays, and raised `max_rp_slew_rate` to `450.0 PWM/s`.
  - **Halved Optical Flow Velocity Filter Lag (`ALPHA_FLOW = 0.50, ALPHA_VEL = 0.55` in `sensor_real.py`):**
    - Reduced combined velocity filter time constant from `~138ms` down to `~60ms`.
- **Validation Status:** All 18 automated unit tests passed (`18/18 OK`). Ready for on-device Raspberry Pi verification.

### Entry 17 — Resolving Growing `2.85s` Oscillations via Sub-Ultimate Velocity Gain (`kp_vel = 0.65`) and Outer-Loop Phase-Lead Decoupling (`kp_pos = 0.35, max_velocity = 0.05`) (2026-11-01)
- **Files Modified:**
  - [`services/drone-inference/cascaded_pid.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/cascaded_pid.py)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`tests/test_inference_altitude.py`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations & Frequency-Domain System Identification (`logs/inference.log`, flight lines 110–838, 24.3s AI flight):**
  1. **What Improved Dramatically in Entry 16:**
     - Compared to Entry 15 (where `shift_x` and `shift_y` diverged to `-0.52m`), Entry 16 kept `shift_y` bounded within **`[-0.100m, +0.116m]` (`±11.6cm`)** and `shift_x` bounded within **`[-0.163m, +0.299m]`** over a full **24.3-second** flight (`t = 5851.15s -> 5875.43s`), including a controlled 6-step descent (`+0.40m -> +0.35m -> +0.30m -> +0.25m -> +0.20m -> +0.14m -> disarm`).
     - Entry 16's bounded return-aware integrator worked as intended: `|I|` stayed within `<= 0.02` (`<= 2 PWM`), and linear regression of horizontal acceleration vs. tilt (`ax` on `Roll - 1500` and `ay` on `Pitch - 1500`) revealed that true neutral hover trim on this airframe is **`1500.1 PWM` (Roll)** and **`1499.4 PWM` (Pitch)** (`~1500 PWM` on both axes).
  2. **Root Cause 1 — Open-Loop Gain Exceeded the Nyquist Ultimate Gain Limit (`Ku = 1.30`) at the `-180°` Phase Crossover Frequency (`w0 ≈ 2.20 rad/s`, `T ≈ 2.85s`):**
     - Fast Fourier Transform (FFT) transfer-function identification of the physical plant $G_v(j\omega) = \frac{V(j\omega)}{U(j\omega)}$ across the 24.3s flight (`lines 145–475` for X, `lines 380–730` for Y) revealed that at $\omega_0 = 2.20\text{ rad/s}$ ($f_0 = 0.35\text{ Hz}$, period $T = 2.85\text{s}$), the combined phase lag of acceleration-to-velocity integration ($-90^\circ$) plus Betaflight Angle Mode attitude response, MSP serial transport, and optical flow filtering ($-92^\circ$) reaches **$-181^\circ$ to $-183^\circ$** with plant gain $|G_v(j\omega_0)| = 0.740\text{–}0.766\text{ (m/s)/unit}$.
     - Consequently, the **ultimate velocity loop gain (Nyquist stability boundary)** at $\omega_0 = 2.20\text{ rad/s}$ is $K_{u,\text{vel}} = \frac{1}{0.766} = \mathbf{1.30\text{ unit/(m/s)}}$.
     - With Entry 16's `kp_vel = 1.20`, the effective controller gain $\left|\frac{U(j\omega_0)}{-V(j\omega_0)}\right|$ measured in the flight log was **`1.305` (X) and `1.351` (Y)**, yielding an open-loop magnitude $|L(j\omega_0)| = 1.31 \times 0.766 = \mathbf{1.004\text{–}1.02 > 1.0}$ right at $-180^\circ$ phase! Because $|L(j\omega_0)|$ was slightly above `1.0`, each half-cycle (`1.43s`) amplified the oscillation by `~10–15%` (`sx` peaks: `-0.043m -> +0.019m -> -0.028m -> +0.032m -> -0.036m -> +0.042m -> -0.109m -> +0.168m -> -0.155m -> +0.299m`; `sy` peaks: `-0.035m -> +0.044m -> -0.046m -> +0.072m -> -0.088m -> +0.097m -> -0.100m -> +0.116m`) until saturating against the `±50 PWM` (`1450 / 1550`) rails.
  3. **Root Cause 2 — Outer Position Loop Phase Lag Cancelled 92% of Inner Velocity D-Term Phase Lead (`kp_pos = 0.60` too high):**
     - The effective linear frequency response of the cascaded controller from $-V(j\omega)$ to $U(j\omega)$ is:
       $$C_{\text{eff}}(j\omega) = \frac{U(j\omega)}{-V(j\omega)} = k_{p,\text{vel}} + j\left(\omega k_{d,\text{vel}} - \frac{k_{p,\text{pos}} k_{p,\text{vel}}}{\omega} - \frac{k_{i,\text{vel}}}{\omega}\right)$$
     - At $\omega_0 = 2.20\text{ rad/s}$, Entry 16's outer position loop introduced a quadrature phase-lag component $-\frac{k_{p,\text{pos}} k_{p,\text{vel}}}{\omega_0} = -\frac{0.60 \times 1.20}{2.20} = \mathbf{-0.327}$, which **cancelled 92% of the phase lead** provided by the inner velocity derivative $+\omega_0 k_{d,\text{vel}} \cos(\phi_d) \approx \mathbf{+0.356}$, leaving only **`+1.0°` (Y) to `+3.2°` (X)** of net phase lead!
     - In the time domain: at every turnaround peak (`v = 0`), the drone was already banked inward (`a ≈ 0.44 m/s²`) from braking the outbound swing; commanding `target_vel = ±0.08 m/s` inward prevented the inner loop from immediately damping that inward acceleration, propelling the drone across origin at `0.27–0.43 m/s`.
  4. **Root Cause 3 — Single-Frame Optical Flow D-Term Chattering (`ALPHA_FLOW = 0.50, ALPHA_VEL = 0.55, d_alpha = 0.45`):**
     - Step changes in SPI optical flow velocity (`Δv ≈ 0.05 m/s` in `33ms`) multiplied by `kd_vel = 0.18` with `d_alpha = 0.45` produced `±12–15 PWM` single-frame D-term kicks (`lines 147–165, 482–488, 521–525, 716–721`).
- **Adjustments Made:**
  - **Halved Inner Loop Proportional Gain Below Nyquist Ultimate Limit (`kp_vel: 1.20 -> 0.65`):**
    - Drops open-loop gain $|L(j\omega_0)|$ from `1.02` down to **`0.50`** (`~6 dB` gain margin below $K_u = 1.30$), ensuring oscillations decay rapidly instead of growing and preventing `1450 / 1550 PWM` bang-bang rail saturation.
  - **Decoupled Outer Position Loop Bandwidth (`kp_pos: 0.60 -> 0.35, max_velocity: 0.08 -> 0.05 m/s`) & Strengthened Net Phase Lead (`kd_vel: 0.18 -> 0.20, d_alpha: 0.45 -> 0.35`):**
    - Cuts the destabilizing outer-loop quadrature lag $\frac{k_{p,\text{pos}} k_{p,\text{vel}}}{\omega_0}$ by **3.2x** (from `-0.327` down to `-0.103`), while $+\omega_0 k_{d,\text{vel}}$ provides `+0.38` of phase lead — boosting net controller phase lead from `+2°` to **`+22°`** at $\omega_0 = 2.20\text{ rad/s}$ and capping return approach speed to a gentle `5 cm/s`.
  - **Reduced Trim Integrator Gain & Bounds (`ki_vel: 0.30 -> 0.15, integral_limit: ±0.35 -> ±5.25 PWM max, max_integral_vel_error: 0.05 m/s`):**
    - Because empirical neutral hover trim is `1500.1 / 1499.4 PWM`, `±5.25 PWM` trim authority is more than sufficient while cutting integral phase lag in half.
  - **Smoothed Optical Flow Velocity & D-Term Filtering (`ALPHA_FLOW = 0.42, ALPHA_VEL = 0.45` in `sensor_real.py`, `d_alpha = 0.35` in `cascaded_pid.py`, `max_rp_slew_rate = 500.0 PWM/s` in `flight_controller.py`):**
    - Suppresses single-frame SPI velocity steps by `35%` while keeping sensor filter lag low (`~85ms` total).
- **Validation Status:** Verified on the FFT-identified closed-loop plant model (confirming monotonic convergence with zero growing oscillations) and passed all 18 automated unit tests (`18/18 OK`). Ready for on-device Raspberry Pi verification.

### Entry 18 — Eliminating Barrier Stalls, D-Term Deceleration Cancellation, and Synchronous Logging Freezes (`kp_pos = 0.45, max_velocity = 0.07, kp_vel = 1.00, ki_vel = 0.22, kd_vel = 0.08`) (2026-11-01)
- **Files Modified:**
  - [`services/drone-inference/cascaded_pid.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/cascaded_pid.py)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`services/drone-inference/inference.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/inference.py)
  - [`core/drone_logging.py`](file:///home/rodion/projects/drone-control-center/core/drone_logging.py)
  - [`tests/test_inference_altitude.py`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 97–713, 22.59s AI flight, 611 frames):**
  1. **What Improved Dramatically in Entry 17:**
     - **Growing `2.85s` resonance completely eliminated:** Unlike Entry 16 (where oscillations grew every `1.4s` until repeatedly saturating the `1450 / 1550 PWM` rails), `Roll` stayed within `[1458, 1536]` during hover (mostly `1480–1520 PWM`) and `Pitch` stayed within `[1464, 1550]` across the full **22.59s** flight.
     - **Pitch (`sy`) Axis Centering:** Across all 599 airborne frames, `sy` achieved **`mean = -0.0055m` (`-5.5 mm`)**, **`median = -0.0001m` (`-0.1 mm`)**, `pct(sy > 0) = 49.9%`, and stayed bounded within `[-0.136m, +0.100m]` (`RMS = 6.8 cm`). During the first 8 seconds of free flight (`t = 0..8s`), `sy` remained within `[-3.7 cm, +7.0 cm]` and settled to `-0.4 cm` (`4 mm`).
     - **Altitude Hold & Step-Down Descent:** Climbed smoothly from `0.018m` to `0.518m`, held `0.36m–0.44m` around the `0.418m` target, and descended briskly from `0.502m` to `0.116m` in `1.1s` when `s` was pressed (`lines 680–712`).
  2. **Root Cause 1 — Starved Static Restoring Authority (`3.25 PWM` max P-pull at `v = 0`) Causing 2.0–3.0s Barrier Stalls (`sx = +0.234m` and `-0.165m`):**
     - With `max_velocity = 0.05 m/s` and `kp_vel = 0.65`, whenever the drone stopped (`vx ≈ 0`) at a displaced position (`|sx| >= 0.15m`), the inner loop's proportional pull was capped at $100 \times 0.65 \times 0.05 = \mathbf{3.25\text{ PWM}}$ (`0.32°` tilt).
     - `3.25 PWM` (`Roll = 1497` or `1503 PWM`) was too weak to overcome airframe roll bias and barrier recirculation, leaving the drone stalled at `sx = +0.231..+0.235m` for **2.0 seconds** (`lines 351–408`) and at `sx = -0.143..-0.167m` for **3.0 seconds** (`lines 471–561`).
  3. **Root Cause 2 — Excessive Acceleration Derivative Ratio ($\tau_d = K_{d,\text{vel}}/K_{p,\text{vel}} = 0.31\text{s}$) Canceling Return Braking & Injecting SPI Chatter:**
     - Reconstructing exact `P`, `I`, and `D` terms across all 599 airborne frames revealed `RMS(D_x) = 8.22 PWM` (nearly equal to `RMS(P_x) = 8.89 PWM`) with **`Mean frame-to-frame |ΔD_x| = 4.46 PWM/frame`** (`3.6x` larger than `|ΔP_x| = 1.25 PWM/frame`).
     - Worse, because $\int_{t_1}^{t_2} D_{\text{vel}}\,dt = -100 K_{d,\text{vel}} (v(t_2) - v(t_1))$, whenever the drone returned toward `sx = 0` and began slowing down (`lines 194–230`, `vx` decelerating from `-0.12 m/s` toward `-0.06 m/s`), $\dot{v}_x > 0$ made $D_x$ systematically negative (`-2.1 to -6.5 PWM`), **canceling out the positive braking tilt `P_x = +3.5..+4.4 PWM`** (`Roll = 1500 PWM`) so the drone stopped braking and coasted right across `sx = 0`.
  4. **Root Cause 3 — Wall-Stall Integral Windup Delaying Rebound Braking & Weak Velocity Braking (`kp_vel = 0.65`):**
     - During the 3.0-second stall at `sx = -0.165m` (`lines 486–561`), `I_x` wound up to `+1.84 PWM` to unstick the drone. Once unstuck (`lines 561–585`), `I_x` remained positive all the way to `sx = 0` (`line 593`) while `kp_vel = 0.65` only generated `65 PWM/(m/s)` of braking, allowing the rebound to accelerate to `+0.35 m/s` and cross to `+0.24m`.
  5. **Root Cause 4 — Synchronous File/Stdout Logging Freezes in 100Hz Flight Loop (`0.314s` at L328, `1.515s` at L681, `0.269s` at L698):**
     - `WatchedFileHandler` (`os.stat()` + `flush()` on microSD card) and `StreamHandler(sys.stdout)` ran synchronously on every frame in `inference.py`. At `line 679` (`thr = 1594 PWM`), a `1.515s` microSD/stdout writeback stall froze `inference.py` while `fc.py` kept sending `thr = 1594 PWM`, causing an uncommanded climb from `0.327m` to `0.451m`.
- **Adjustments Made:**
  - **Restored Balanced Cascaded Authority (`kp_pos = 0.45, max_velocity = 0.07 m/s, kp_vel = 1.00, ki_vel = 0.22`):**
    - Increases static proportional pull at `v = 0` by **2.15x** (`7.0 PWM` vs `3.25 PWM`, plus up to `8.8 PWM` integral = `15.8 PWM` unsticking authority) and increases velocity braking authority by **54%** (`100 PWM/(m/s)` vs `65 PWM/(m/s)`).
    - Keeps effective position stiffness near origin at $K_{\text{pos,eff}} = 45.0\text{ PWM/m}$ (`2.5x` below Entry 16's resonant `112 PWM/m`) while raising the closed-loop damping ratio $\zeta \propto \sqrt{K_{p,\text{vel}} / K_{p,\text{pos}}}$ to its highest value yet ($\sqrt{1.00/0.45} = 1.49$).
  - **Eliminated D-Term Deceleration Cancellation & SPI Chatter (`kd_vel: 0.20 -> 0.08, d_alpha: 0.35 -> 0.25`):**
    - Sets inner acceleration lead time constant $\tau_d = K_{d,\text{vel}} / K_{p,\text{vel}} = 0.08\text{ s}$ (`80 ms`, matching Betaflight attitude actuator response), cutting frame-to-frame D-term chatter by **3.5x** (`4.46 -> 1.27 PWM/frame`) and ensuring `P_vel` braking stays active (`+4 to +5 PWM`) all the way to origin on return.
  - **Implemented Distance-Proportional Return Integral Unwind (`cascaded_pid.py`):**
    - Whenever the drone is moving toward the target (`position_error * current_vel > 0`), any return-direction integral (`vel_pid.integral * position_error > 0`) never winds up further and unwinds proportional to distance traveled (`-3.0 * current_vel * dt`, clamped at zero-crossing) so wall-unsticking integral sheds cleanly before reaching origin.
  - **Offloaded Logging to Non-Blocking Bounded QueueListener (`core/drone_logging.py`) & Non-Blocking ZMQ PUB (`inference.py`):**
    - Wrapped `WatchedFileHandler` and `StreamHandler` in a standard-library `QueueListener` with a bounded `queue.Queue(maxsize=512)` and `_NonBlockingQueueHandler` (`put_nowait`), and added `flags=zmq.NOBLOCK` to `rc_pub.send_pyobj(...)`, eliminating SD card / stdout freezes in the 100Hz flight loop.
- **Validation Status:** Verified by replaying `logs/inference.log` through the Entry 18 controller and passed all 18 automated unit tests (`18/18 OK`). Ready for on-device Raspberry Pi verification.

### Entry 19 — Eliminating Rapid Oscillation Growth, Right-Wall Gluing (`Ix` Unwind Starvation), and Optical Flow Stall-Burst Spikes (`kp_pos = 0.45, max_velocity = 0.08, kp_vel = 0.72, ki_vel = 1.10, kd_vel = 0.16`) (2026-11-01)
- **Files Modified:**
  - [`services/drone-sensors/sensor_real.py`](file:///home/rodion/projects/drone-control-center/services/drone-sensors/sensor_real.py)
  - [`services/drone-inference/cascaded_pid.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/cascaded_pid.py)
  - [`services/drone-inference/flight_controller.py`](file:///home/rodion/projects/drone-control-center/services/drone-inference/flight_controller.py)
  - [`tests/test_inference_altitude.py`](file:///home/rodion/projects/drone-control-center/tests/test_inference_altitude.py)
  - [`docs/shift_pid_testing_handoff.md`](file:///home/rodion/projects/drone-control-center/docs/shift_pid_testing_handoff.md)
  - [`DESIGN.md`](file:///home/rodion/projects/drone-control-center/DESIGN.md)
- **Telemetry Observations (`logs/inference.log`, flight lines 96–343, 8.91s AI flight, 241 frames):**
  1. **Flight Trajectory Summary:**
     - **Swing 1 (`t = 0.48s -> 2.20s`, L111–L162):** Drone lifts off and climbs to `0.510m`. Even though `Roll` averages `1490.1 PWM` (`-10 PWM` left bank), the drone drifts right from `sx = 0.000m` to `sx = +0.102m` (`vx ≈ +0.08..+0.11 m/s`), confirming the airframe's physical neutral Roll hover trim is **`~1489–1490 PWM` (`-10 to -11 PWM`)**.
     - **Swing 2 (`t = 2.20s -> 3.55s`, L162–L202):** From `sx = +0.102m`, the drone accelerates left to **`vx = -0.339 m/s`** (`L185`) and overshoots across origin to **`sx = -0.150m`**.
     - **Swing 3 (`t = 3.58s -> 4.63s`, L203–L210):** Rebounds from `sx = -0.150m` all the way across the cage to **`sx = +0.150m`** (colliding with the right wall at `t = 4.63s`).
     - **Right-Wall Gluing (`t = 4.63s -> 8.60s`, L210–L332, 3.97 seconds):** The drone remains **glued to the right wall** at `sx = +0.115m..+0.165m` (`mean = +0.145m`, `L254–L318` `sx = +0.151m -> +0.156m`, `mean vx = +0.003 m/s`) until the operator steps down `shift_z` 5 times (`L292..L333`) and disarms (`L343`).
  2. **Root Cause 1 — Why the Drone Got Glued to the Right Wall (`L210–L332`): Flawed Return-Unwind Condition (`position_error * current_vel > 0`) & Starved Trim Integration (`Ix` stuck at `-0.47 PWM` mean, `-2.03 PWM` min):**
     - Reconstructing exact `Px`, `Ix`, `Dx` across all 241 frames showed `Roll Ix` **never exceeded `-2.03 PWM`** (`mean = -0.47 PWM`, `final = 0.00 PWM`).
     - While pinned against the right wall (`sx = +0.15m`, `position_error = -0.15m`), `target_velocity = -0.07 m/s` produced `Px = -7.0 PWM` (`Roll = 1493 PWM`). Because the airframe's neutral Roll hover trim (`~1489.5 PWM`) plus wall propwash Coanda suction requires `~1485–1487 PWM` just for zero lateral acceleration, **`Roll = 1493 PWM` was still tilted +6 to +8 PWM to the RIGHT of equilibrium**, pressing the drone continuously into the right wall.
     - Why didn't `Ix` integrate down to `-12..-15 PWM` to unstick the drone?
       1. In `cascaded_pid.py` line 79, `if self.position_error * current_vel > 0.0:` triggered whenever `current_vel` had **any** negative value — even `-0.00003 m/s` (decaying LPF tail while 100% motionless on the wall at `L281–L291`) or `-0.015 m/s` (`8mm` wiggle at `L226–L229` and `L256–L272`), **even though `|current_vel| < |target_velocity|` (`velocity_error < 0`)**.
       2. That branch blocked integration and executed `self.vel_pid.integral -= 3.0 * current_vel * dt` (`-66 PWM/m`), which **wiped `Ix` completely to `0.0 PWM`** after just `8 mm` of leftward wiggle (`L229` and `L272`) and during the initial climb (`L163–L171`).
       3. Moreover, with `ki_vel = 0.22` and `max_integral_vel_error = 0.07`, the maximum possible integral build rate at `v = 0` was only `100 * 0.22 * 0.07 = 1.54 PWM/s` (requiring `>7 seconds` to learn a `-11 PWM` trim offset).
  3. **Root Cause 2 — Why the Oscillation Grew Fast (`t = 0.5s -> 4.6s`): High `kp_vel = 1.00` + Weak `kd_vel = 0.08` ($\tau_d = 80\text{ms}$) Destroyed Phase Margin:**
     - In Entry 17 (`kp_vel = 0.65, kd_vel = 0.20, d_alpha = 0.35`, $\tau_d = 0.31\text{s}$), the drone flew 22.6s with zero growing oscillations.
     - Raising `kp_vel` to `1.00` while slashing `kd_vel` to `0.08` (`d_alpha = 0.25`) reduced controller phase lead by `~30°` and raised open-loop gain by `60%`. Weak `kd_vel = 0.08` failed to damp leftward acceleration at turnaround (`L162`, `vx` reached `-0.339 m/s`) and failed to level the wings while decelerating (`L180–L195`), leaving `Roll` at `1530–1539 PWM` for `0.5s` after crossing `sx = 0`.
  4. **Root Cause 3 — Optical Flow Stall-Burst Filter Smearing in `sensor_real.py` Causing a Fake `+2.43 m/s` Velocity Explosion at `L205`:**
     - Between `L203` (`1793559798.013`) and `L204` (`1793559798.856`), `drone-sensors` stalled for `842.9ms` while the drone moved right from `sx = -0.148m` to `+0.050m` (`+19.8 cm` over `0.84s` = normal `+0.23 m/s`).
     - Because `sensor_real.py` updated `self.last_time` before `get_motion()` and fed the multi-frame hardware-accumulated pixel burst (`dx` over `843ms`) directly into the per-sample IIR filter `self.filtered_dx`, on the very next frame `37.1ms` later (`L205`), `self.filtered_dx` still retained `58%` of the 843ms pixel burst and divided it by `dt = 0.0371s` — producing a bogus **`vx = +2.425 m/s` (`obs[4] = +0.485`)** spike that slammed `Roll` to the `1450 PWM` rail (`L207–L209`).
- **Adjustments Made:**
  - **Fixed Trim Integrator Condition (`position_error * velocity_error >= 0`) & Unsticking Authority (`ki_vel = 1.10, integral_limit = ±0.15 -> ±16.5 PWM`) in `cascaded_pid.py`:**
    - Replaced `if self.position_error * current_vel > 0.0:` with `if self.position_error * self.velocity_error >= 0.0:`. Whenever the drone is drifting away, stalled against a wall, or moving toward the target **slower** than `target_velocity` (`|current_vel| <= |target_velocity|`), it continues integrating `clamped_vel_err * dt` (with a `1.5x` boost when displaced `> 6 cm` and moving `< 50%` of `target_velocity`) and **never unwinds learned trim on small return wiggles**.
    - Raised `ki_vel` to `1.10` with `integral_limit = ±0.15` (`±16.5 PWM` max trim, up to `8.8–13.2 PWM/s` build rate when stalled), allowing the controller to learn the `-10.5 PWM` neutral Roll hover trim within `~1.2s` and peel off wall Coanda suction within `< 1.0s`.
    - Only when returning toward target **faster** than `target_velocity` (`position_error * velocity_error < 0`) does the controller freeze opposing integral windup and gently relax excess return-direction integral (`0.35x` for normal speed, `2.5x` for high excess speed `|v| > 1.5 * max_velocity`).
  - **Restored Sub-Ultimate Inner Velocity Gain & Turnaround Damping Lead (`kp_pos = 0.45, max_velocity = 0.08 m/s, kp_vel = 0.72, kd_vel = 0.16, d_alpha = 0.32`):**
    - `kp_vel = 0.72` stays well below the Nyquist ultimate gain ($K_u = 1.30$, open-loop gain $\approx 0.55 < 1.0$) while $\tau_d = K_{d,\text{vel}} / K_{p,\text{vel}} = 0.16 / 0.72 = \mathbf{0.222\text{ s}}$ (`222 ms`) restores `+18°` of phase lead at $\omega_0 = 2.2\text{ rad/s}$, damping turnaround acceleration early and leveling the wings before crossing origin.
  - **Fixed Multi-Frame Stall Normalization & Physical Velocity Clamping in `sensor_real.py`:**
    - Compute `dt = max(0.010, current_time - self.last_time)` only after `self.flow.get_motion(timeout=0.025)` returns valid motion, normalize multi-frame stall bursts (`dt > 0.10s`) by `NOMINAL_FLOW_DT / dt` (`0.035 / dt`) before `self.filtered_dx` / `self.filtered_dy` so stall bursts never smear into subsequent `35ms` velocity frames, and clamp single-frame steps (`MAX_FRAME_SHIFT_M = 0.030m`) and physical indoor velocity (`MAX_PHYSICAL_VEL_MS = 0.60 m/s`).
- **Validation Status:** Verified in closed-loop physics simulation (with `+10.5 PWM` physical roll asymmetry, `+2.5 PWM` wall Coanda suction, `120ms` attitude lag, and sensor jitter) and passed all 19 automated unit tests (`19/19 OK`, including `test_cascaded_wall_unsticking_preserves_trim_under_small_return_wiggles`). Ready for on-device Raspberry Pi verification.
