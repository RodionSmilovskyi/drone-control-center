import numpy as np
from pid_controller import PIDController

class FlightController:
    """Low-level controller translating high-level actions to RC commands."""
    def __init__(self):
        self.throttle_pid = PIDController(Kp=4, Ki=0, Kd=2)
        
        self.hover_throttle = 1421
        self.min_throttle = 1341
        self.max_throttle = 1491
        
        self.reset()

    def reset(self):
        self.throttle_pid.reset()
    
    def compute_rc_commands(self, high_level_action: np.ndarray, current_alt_norm: float, dt: float) -> np.ndarray:
        # high_level_action: [desired_alt, desired_roll, desired_pitch, desired_yaw_rate]
        # action[0] is in [-1, 1], remap it to [0, 1] for altitude setpoint
        desired_alt_norm = (high_level_action[0] + 1) / 2
        desired_roll_norm, desired_pitch_norm, desired_yaw_rate_norm = high_level_action[1:]
        
        self.throttle_pid.setpoint = desired_alt_norm
        throttle_pid_out = self.throttle_pid.compute(current_alt_norm, dt)
        
        rc_throttle = self.hover_throttle + (100 * throttle_pid_out)

        # Map desired roll, pitch, and yaw rate from [-1, 1] to [1000, 2000]
        rc_roll = 1500 + 500 * desired_roll_norm
        rc_pitch = 1500 + 500 * desired_pitch_norm
        rc_yaw = 1500 + 500 * desired_yaw_rate_norm

        rc_commands = np.clip([rc_throttle, rc_roll, rc_pitch, rc_yaw], 1000, 2000)
        rc_commands[0] = np.clip(rc_commands[0], self.min_throttle, self.max_throttle)
        
        return rc_commands.astype(np.float32)
