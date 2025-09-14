import numpy as np
import numpy.typing as npt
import time
from pid_controller import PIDController



class FlightController:
    """
    This is the "Skillful Driver" (the low-level controller).
    It takes high-level commands and the current state (both normalized) from the RL agent
    and translates them into low-level RC commands.
    This version is fully self-contained and calculates all necessary rates internally.
    """
    def __init__(self):
        # --- PID Controllers for each axis ---
        self.throttle_pid = PIDController(Kp=15, Ki=0, Kd=5)
        self.roll_pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.2)
        self.pitch_pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.2)
        self.yaw_pid = PIDController(Kp=1.5, Ki=0.0, Kd=1)
        self.ff_throttle = 1260
        
        self.reset()

    def reset(self):
        """Resets all PID controllers and state variables."""
        self.throttle_pid.reset()
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
    
    def compute_rc_commands(self, high_level_action: np.ndarray, state_goal: np.ndarray, dt: float) -> np.ndarray:
        """
        Converts the RL agent's high-level desires into low-level RC commands.
        """
        current_alt_norm, current_roll_norm, current_pitch_norm, current_yaw_norm, _ = state_goal
        desired_alt_norm, desired_roll_norm, desired_pitch_norm, desired_yaw_norm = high_level_action
        
        self.throttle_pid.setpoint = desired_alt_norm
        throttle_command = self.throttle_pid.compute(current_alt_norm, dt)
        
        self.roll_pid.setpoint = desired_roll_norm
        roll_command = self.roll_pid.compute(current_roll_norm, dt)
        
        self.pitch_pid.setpoint = desired_pitch_norm
        pitch_command = self.pitch_pid.compute(current_pitch_norm, dt)
        
        clockwise_yaw_distance = abs(desired_yaw_norm - current_yaw_norm)
        counterclockwise_yaw_distance = abs((desired_yaw_norm - 1) - current_yaw_norm)
        
        if clockwise_yaw_distance < counterclockwise_yaw_distance:
            self.yaw_pid.setpoint = desired_yaw_norm
        else:
            self.yaw_pid.setpoint = desired_yaw_norm - 1
        yaw_command = self.yaw_pid.compute(current_yaw_norm, dt)
        
        # --- Convert to [1000, 2000] RC Command Range ---
        rc_throttle = 1260 + 500 * throttle_command
        rc_roll = 1500 + 500 * roll_command
        rc_pitch = 1500 + 500 * pitch_command
        rc_yaw = 1500 + 500 * yaw_command

        rc_commands = np.clip([rc_throttle, rc_roll, rc_pitch, rc_yaw], 1000, 2000)

        return rc_commands.astype(int)

