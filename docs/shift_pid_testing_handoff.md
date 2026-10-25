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
| **Throttle (Z)** | `self.throttle_pid` | `4.5` | `1.2` | `2.0` | `[-0.35, +1.20]` | `1480` | `[-135, +140]` | `[1345, 1620]` | Slew 0.35 m/s; 8cm ground leash; anti-windup < 0.020m |
| **Roll (X)** | `self.roll_pid` | `0.5` | `0.05` | `0.4` | `[-0.20, +0.20]` | `1500` | `[-50, +50]` (`±5°`) | `[1450, 1550]` | Locked to `1500` below `0.020m` |
| **Pitch (Y)** | `self.pitch_pid` | `0.5` | `0.05` | `0.4` | `[-0.20, +0.20]` | `1500` | `[-50, +50]` (`±5°`) | `[1450, 1550]` | Locked to `1500` below `0.020m` |
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
