import numpy as np
from pid_controller import PIDController

class FlightController:
    """Low-level controller translating high-level actions to RC commands."""
    def __init__(self):
        # Confined-space tuning with soft takeoff setpoint ramping:
        # Kp=8.0, Ki=0.5, Kd=2.2 with bounded integral authority (+/-20 PWM) to eliminate windup cycling
        self.throttle_pid = PIDController(Kp=8.0, Ki=0.5, Kd=2.2, integral_limit=0.4)
        
        self.hover_throttle = 1625
        self.min_throttle = 1341
        self.max_throttle = 1800
        self.max_pid_correction = 100.0  # Gives throttle range [1525, 1725] to compensate for battery sag
        
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
        # action[0] is in [-1, 1], remap it to [0, 1] for altitude setpoint
        desired_alt_norm = (high_level_action[0] + 1) / 2
        desired_roll_norm, desired_pitch_norm, desired_yaw_rate_norm = high_level_action[1:]
        
        # Smooth setpoint slew to eliminate catapult takeoff and abrupt steps
        if self.current_setpoint_norm is None:
            self.current_setpoint_norm = current_alt_norm
        else:
            max_step = self.max_climb_rate_norm * dt
            delta = desired_alt_norm - self.current_setpoint_norm
            self.current_setpoint_norm += np.clip(delta, -max_step, max_step)
        
        self.throttle_pid.setpoint = self.current_setpoint_norm
        
        # Anti-windup: only accumulate integral once airborne past ground threshold
        enable_integral = current_alt_norm >= self.ground_threshold_norm
        throttle_pid_out = self.throttle_pid.compute(current_alt_norm, dt, enable_integral=enable_integral)
        
        # Bound PID correction to prevent explosive acceleration/deceleration in enclosed spaces
        pid_correction = np.clip(100.0 * throttle_pid_out, -self.max_pid_correction, self.max_pid_correction)
        rc_throttle = self.hover_throttle + pid_correction

        # Map desired roll, pitch, and yaw rate from [-1, 1] to [1000, 2000]
        rc_roll = 1500 + 500 * desired_roll_norm
        rc_pitch = 1500 + 500 * desired_pitch_norm
        rc_yaw = 1500 + 500 * desired_yaw_rate_norm

        rc_commands = np.clip([rc_throttle, rc_roll, rc_pitch, rc_yaw], 1000, 2000)
        rc_commands[0] = np.clip(rc_commands[0], self.min_throttle, self.max_throttle)
        
        return rc_commands.astype(np.float32)
