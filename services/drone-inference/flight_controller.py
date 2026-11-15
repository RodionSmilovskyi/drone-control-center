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

        # iNAV deadbands (matching CLI: alt_hold_deadband = 40, pos_hold_deadband = 25)
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
