import numpy as np
from pid_controller import PIDController

class FlightController:
    """Low-level controller translating high-level actions to RC commands."""
    def __init__(self):
        # Confined-space tuning paired with Betaflight vbat_sag_compensation:
        # Hover baseline 1535 PWM ensures prompt, gentle unweighting (~1570 PWM at t=0.6s) without catapulting.
        # Integral trim (+185 PWM headroom -> 1720 PWM ceiling) briskly elevates throttle for sagged batteries.
        self.throttle_pid = PIDController(Kp=5.2, Ki=1.8, Kd=2.2, integral_limit=1.35, integral_min=-0.35, d_alpha=0.6)
        # Horizontal PIDs: Overdamped velocity-braking gains (Kp=0.75, Ki=0.35, Kd=0.85, integral_limit=±0.50 -> ±17.5 PWM max trim,
        # d_alpha=0.35, anti_windup_return=True). Kd > Kp caps return velocity to ~10-15 cm/s so the drone never accelerates into
        # the opposite barrier, while Ki=0.35 + anti_windup_return rapidly unsticks the drone if stalled against a barrier.
        self.roll_pid = PIDController(Kp=0.75, Ki=0.35, Kd=0.85, integral_limit=0.50, integral_min=-0.50, d_alpha=0.35, anti_windup_return=True)
        self.pitch_pid = PIDController(Kp=0.75, Ki=0.35, Kd=0.85, integral_limit=0.50, integral_min=-0.50, d_alpha=0.35, anti_windup_return=True)
        self.yaw_pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.1, integral_limit=0.2, integral_min=-0.2)
        
        self.hover_throttle = 1535
        self.min_throttle = 1341
        self.max_throttle = 1800
        # Expanded descent authority [-150] allows throttle down to 1385 PWM so drone can brake and descend on fresh packs,
        # headroom [+185] allows throttle up to 1720 PWM as battery voltage sags across flights.
        self.max_climb_correction = 185.0
        self.max_descent_correction = -150.0
        self.max_pid_correction = self.max_climb_correction
        
        # Explicit degree-to-PWM indoor tilt authority limits (SpeedyBee max angle ~50 deg -> 500 PWM)
        self.fc_max_angle_deg = 50.0
        self.max_indoor_tilt_deg = 5.0
        self.max_rp_correction = (self.max_indoor_tilt_deg / self.fc_max_angle_deg) * 500.0
        self.max_rp_slew_rate = 350.0  # Max 350 PWM/s (~11.5 PWM per 33ms frame) to avoid unwind phase lag at turnaround
        self.max_yaw_correction = 50.0
        
        # Ground threshold for throttle integral: 0.018m (below resting ~0.021-0.030m ToF reading)
        # so throttle_pid can integrate past 1601 PWM to lift off when battery voltage sags.
        self.ground_threshold_norm = 0.018 / 3.0
        # Airborne threshold for horizontal Roll/Pitch/Yaw PIDs: 0.050m (above ~0.044m ground unweighting)
        # so Roll/Pitch stay strictly neutral (1500 PWM) with zero integral windup until genuinely airborne.
        self.xy_airborne_threshold_norm = 0.050 / 3.0
        
        # Slew rate: 0.35 m/s (~0.117 normalized units/sec) to smoothly lead climb without ceiling overshoot
        self.max_climb_rate_norm = 0.35 / 3.0
        self.current_setpoint_norm = None
        self.ref_alt_norm = None
        self.ref_shift_x_norm = None
        self.ref_shift_y_norm = None
        self.last_roll_correction = 0.0
        self.last_pitch_correction = 0.0
        
        self.reset()

    def reset(self):
        self.throttle_pid.reset()
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.current_setpoint_norm = None
        self.ref_alt_norm = None
        self.ref_shift_x_norm = None
        self.ref_shift_y_norm = None
        self.last_roll_correction = 0.0
        self.last_pitch_correction = 0.0
    
    def compute_rc_commands(self, high_level_action: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        # high_level_action: [desired_shift_z, desired_shift_x, desired_shift_y, desired_yaw_norm] in [-1, 1]
        # current_state: [current_alt_norm, current_shift_x_norm, current_shift_y_norm, current_yaw_norm]
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

        is_throttle_active = current_alt_norm >= self.ground_threshold_norm
        is_xy_airborne = current_alt_norm >= self.xy_airborne_threshold_norm

        # Latch reference state when AI mode engages (first frame after reset)
        if self.ref_alt_norm is None:
            self.ref_alt_norm = current_alt_norm
        if self.ref_shift_x_norm is None:
            self.ref_shift_x_norm = current_shift_x_norm if is_xy_airborne else 0.0
        if self.ref_shift_y_norm is None:
            self.ref_shift_y_norm = current_shift_y_norm if is_xy_airborne else 0.0

        # On the ground before takeoff (< 0.050m), horizontal origin is always zero
        if not is_xy_airborne:
            self.ref_shift_x_norm = 0.0
            self.ref_shift_y_norm = 0.0

        desired_alt_norm = float(np.clip(self.ref_alt_norm + desired_shift_z, 0.0, 1.0))
        desired_x_norm = float(np.clip(self.ref_shift_x_norm + desired_shift_x, -1.0, 1.0))
        desired_y_norm = float(np.clip(self.ref_shift_y_norm + desired_shift_y, -1.0, 1.0))

        # Smooth setpoint slew to eliminate catapult takeoff and abrupt steps
        clamped_dt = min(max(dt, 0.001), 0.05)
        if self.current_setpoint_norm is None:
            self.current_setpoint_norm = current_alt_norm
        else:
            max_step = self.max_climb_rate_norm * clamped_dt
            delta = desired_alt_norm - self.current_setpoint_norm
            self.current_setpoint_norm += float(np.clip(delta, -max_step, max_step))

        self.throttle_pid.setpoint = self.current_setpoint_norm
        self.roll_pid.setpoint = desired_x_norm
        self.pitch_pid.setpoint = desired_y_norm
        self.yaw_pid.setpoint = desired_yaw_norm
        
        # Allow throttle integral above 0.018m so the drone can ramp past 1601 PWM to break ground contact.
        throttle_pid_out = self.throttle_pid.compute(current_alt_norm, dt, enable_integral=is_throttle_active)
        
        # Asymmetrical PID bounds: descent authority [-150], climb headroom [+165] (throttle up to 1700 PWM)
        pid_correction = np.clip(100.0 * throttle_pid_out, self.max_descent_correction, self.max_climb_correction)
        rc_throttle = self.hover_throttle + pid_correction

        # Compute Roll, Pitch, and Yaw PIDs; keep neutral (1500 PWM) and integral disabled until > 0.050m
        roll_pid_out = self.roll_pid.compute(current_shift_x_norm, dt, enable_integral=is_xy_airborne)
        pitch_pid_out = self.pitch_pid.compute(current_shift_y_norm, dt, enable_integral=is_xy_airborne)
        yaw_pid_out = self.yaw_pid.compute(current_yaw_norm, dt, enable_integral=is_xy_airborne)

        if is_xy_airborne:
            target_roll_corr = float(np.clip(100.0 * roll_pid_out, -self.max_rp_correction, self.max_rp_correction))
            target_pitch_corr = float(np.clip(100.0 * pitch_pid_out, -self.max_rp_correction, self.max_rp_correction))
            max_rp_step = self.max_rp_slew_rate * clamped_dt
            roll_correction = self.last_roll_correction + float(np.clip(target_roll_corr - self.last_roll_correction, -max_rp_step, max_rp_step))
            pitch_correction = self.last_pitch_correction + float(np.clip(target_pitch_corr - self.last_pitch_correction, -max_rp_step, max_rp_step))
            yaw_correction = np.clip(100.0 * yaw_pid_out, -self.max_yaw_correction, self.max_yaw_correction)
        else:
            roll_correction = 0.0
            pitch_correction = 0.0
            yaw_correction = 0.0

        self.last_roll_correction = roll_correction
        self.last_pitch_correction = pitch_correction

        rc_roll = 1500.0 + roll_correction
        rc_pitch = 1500.0 + pitch_correction
        rc_yaw = 1500.0 + yaw_correction

        rc_commands = np.clip([rc_throttle, rc_roll, rc_pitch, rc_yaw], 1000, 2000)
        rc_commands[0] = np.clip(rc_commands[0], self.min_throttle, self.max_throttle)
        
        return rc_commands.astype(np.float32)
