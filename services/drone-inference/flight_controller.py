import numpy as np
from pid_controller import PIDController

class FlightController:
    """Low-level controller translating high-level actions to RC commands."""
    def __init__(self):
        # Confined-space tuning paired with Betaflight vbat_sag_compensation:
        # True hover thrust with sag compensation is ~1555-1565 PWM
        self.throttle_pid = PIDController(Kp=5.0, Ki=0.4, Kd=2.8, integral_limit=1.2)
        
        self.hover_throttle = 1555
        self.min_throttle = 1341
        self.max_throttle = 1800
        # Asymmetrical authority: tight descent authority [-50] to eliminate ground-effect bounces,
        # expanded climb headroom [+130] for battery sag compensation (throttle range [1505, 1685])
        self.max_climb_correction = 130.0
        self.max_descent_correction = -50.0
        self.max_pid_correction = self.max_climb_correction
        
        # Ground threshold: ~0.020m (landing gear height ~0.013m) to enable integral once unweighted
        self.ground_threshold_norm = 0.020 / 3.0
        
        # Max climb/descent slew rate: 0.25 m/s (~0.083 normalized units/sec) to eliminate takeoff catapult
        self.max_climb_rate_norm = 0.25 / 3.0
        self.current_setpoint_norm = None
        
        self.reset()

    def reset(self):
        self.throttle_pid.reset()
        self.current_setpoint_norm = None
    
    def compute_rc_commands(self, high_level_action: np.ndarray, current_alt_norm: float, dt: float) -> np.ndarray:
        # high_level_action: [desired_alt, desired_roll, desired_pitch, desired_yaw_rate]
        # desired_alt is in [-1, 1], map to [0, 1] normalized altitude
        desired_alt_norm = (high_level_action[0] + 1.0) / 2.0
        desired_roll_norm = high_level_action[1]
        desired_pitch_norm = high_level_action[2]
        desired_yaw_rate_norm = high_level_action[3]

        # Smooth setpoint slew to eliminate catapult takeoff and abrupt steps
        clamped_dt = min(max(dt, 0.001), 0.05)
        if self.current_setpoint_norm is None:
            self.current_setpoint_norm = current_alt_norm
        else:
            max_step = self.max_climb_rate_norm * clamped_dt
            delta = desired_alt_norm - self.current_setpoint_norm
            self.current_setpoint_norm += np.clip(delta, -max_step, max_step)
        
        self.throttle_pid.setpoint = self.current_setpoint_norm
        
        # Anti-windup: accumulate integral once airborne past ground threshold.
        # Below setpoint: allow continuous integral accumulation so hover throttle adapts as battery sags.
        # Above setpoint: freeze integral if floating > 8cm over target to prevent negative windup.
        is_airborne = current_alt_norm >= self.ground_threshold_norm
        below_or_near = (current_alt_norm <= self.current_setpoint_norm) or (abs(current_alt_norm - self.current_setpoint_norm) <= (0.08 / 3.0))
        enable_integral = is_airborne and below_or_near
        throttle_pid_out = self.throttle_pid.compute(current_alt_norm, dt, enable_integral=enable_integral)
        
        # Asymmetrical PID bounds: tight descent authority [-50] to eliminate ground bounce,
        # expanded climb headroom [+130] for battery sag compensation (throttle up to ~1685 PWM)
        pid_correction = np.clip(100.0 * throttle_pid_out, self.max_descent_correction, self.max_climb_correction)
        rc_throttle = self.hover_throttle + pid_correction

        # Map desired roll, pitch, and yaw rate from [-1, 1] to [1000, 2000]
        rc_roll = 1500 + 500 * desired_roll_norm
        rc_pitch = 1500 + 500 * desired_pitch_norm
        rc_yaw = 1500 + 500 * desired_yaw_rate_norm

        rc_commands = np.clip([rc_throttle, rc_roll, rc_pitch, rc_yaw], 1000, 2000)
        rc_commands[0] = np.clip(rc_commands[0], self.min_throttle, self.max_throttle)
        
        return rc_commands.astype(np.float32)
