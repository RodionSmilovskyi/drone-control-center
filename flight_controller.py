import numpy as np
from pid_controller import PIDController

class FlightController:
    """Low-level controller translating high-level actions to RC commands."""
    def __init__(self):
        self.throttle_pid = PIDController(Kp=4, Ki=0, Kd=2)
        self.roll_pid = PIDController(Kp=2, Ki=0.1, Kd=0.5)
        self.pitch_pid = PIDController(Kp=2, Ki=0.1, Kd=0.5)
        self.yaw_pid = PIDController(Kp=2, Ki=1, Kd=0)
        
        self.hover_throttle = 1421
        self.min_throttle = 1341
        self.max_throttle = 1491
        self.min_roll_pitch = 1300
        self.max_roll_pitch = 1700
        
        self.reset()

    def reset(self):
        self.throttle_pid.reset()
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
    
    def compute_rc_commands(self, high_level_action: np.ndarray, state_goal: np.ndarray, dt: float) -> np.ndarray:
        # state_goal: [altitude, roll, pitch, yaw_rate]
        current_alt_norm, current_roll_norm, current_pitch_norm, current_yaw_rate_norm = state_goal
        # high_level_action: [desired_alt, desired_roll, desired_pitch, desired_yaw_rate]
        # action[0] is in [-1, 1], remap it to [0, 1] for altitude setpoint
        desired_alt_norm = (high_level_action[0] + 1) / 2
        desired_roll_norm, desired_pitch_norm, desired_yaw_rate_norm = high_level_action[1:]
        
        self.throttle_pid.setpoint = desired_alt_norm
        throttle_pid_out = self.throttle_pid.compute(current_alt_norm, dt)
        
        rc_throttle = self.hover_throttle + (100 * throttle_pid_out)

        self.roll_pid.setpoint = desired_roll_norm
        roll_command = self.roll_pid.compute(current_roll_norm, dt)
        
        self.pitch_pid.setpoint = desired_pitch_norm
        pitch_command = self.pitch_pid.compute(current_pitch_norm, dt)
        
        self.yaw_pid.setpoint = desired_yaw_rate_norm
        yaw_command = self.yaw_pid.compute(current_yaw_rate_norm, dt)
        
        rc_roll = 1500 + 20 * roll_command
        rc_pitch = 1500 + 20 * pitch_command
        rc_yaw = 1500 + 20 * yaw_command
        
        rc_roll = np.clip(rc_roll, self.min_roll_pitch, self.max_roll_pitch)
        rc_pitch = np.clip(rc_pitch, self.min_roll_pitch, self.max_roll_pitch)

        rc_commands = np.clip([rc_throttle, rc_roll, rc_pitch, rc_yaw], 1000, 2000)
        rc_commands[0] = np.clip(rc_commands[0], self.min_throttle, self.max_throttle)
        
        return rc_commands.astype(np.float32)
