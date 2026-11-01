import numpy as np
from pid_controller import PIDController

class CascadedAxisController:
    """
    Cascaded Position (P) -> Velocity (PI/PID) Controller (PX4 / ArduPilot / iNAV style).
    
    Architecture:
      Outer Loop: Position error (target_pos - current_pos) -> target_velocity (clamped to [-max_velocity, max_velocity])
      Inner Loop: Velocity error (target_velocity - current_vel) -> tilt correction (PWM)
    
    Advantages over single-loop position PID:
      1. Bounded approach speed: drone never accelerates out of control during large setpoint steps (e.g. +0.3m shifts).
      2. Direct velocity damping: 1st-order plant eliminates -180 deg phase lag and toilet-bowling.
      3. Zero position integral windup: inner loop integrates velocity error, naturally learning hover trim
         (e.g., ~1490 PWM Roll) without winding up during position swings or arbitrary waypoint shifts.
    """
    def __init__(
        self,
        kp_pos: float = 0.45,
        max_velocity: float = 0.07,
        kp_vel: float = 1.00,
        ki_vel: float = 0.22,
        kd_vel: float = 0.08,
        integral_limit: float = 0.40,
        integral_min: float = -0.40,
        d_alpha: float = 0.25,
        max_integral_vel_error: float = 0.07,
    ):
        self.kp_pos = kp_pos
        self.max_velocity = max_velocity
        self.max_integral_vel_error = max_integral_vel_error
        self.target_position = 0.0
        self.target_velocity = 0.0
        self.position_error = 0.0
        self.velocity_error = 0.0
        
        self.vel_pid = PIDController(
            Kp=kp_vel,
            Ki=ki_vel,
            Kd=kd_vel,
            integral_limit=integral_limit,
            integral_min=integral_min,
            d_alpha=d_alpha,
            anti_windup_return=False,
        )

    def reset(self):
        self.target_position = 0.0
        self.target_velocity = 0.0
        self.position_error = 0.0
        self.velocity_error = 0.0
        self.vel_pid.reset()

    def compute(
        self,
        current_pos: float,
        current_vel: float,
        dt: float,
        target_pos: float = 0.0,
        enable_integral: bool = True,
    ) -> float:
        self.target_position = target_pos
        self.position_error = self.target_position - current_pos
        
        # Outer Loop: P-control with hard velocity clamp
        self.target_velocity = float(np.clip(self.kp_pos * self.position_error, -self.max_velocity, self.max_velocity))
        self.vel_pid.setpoint = self.target_velocity
        self.velocity_error = self.target_velocity - current_vel
        
        # Bounded, return-aware trim integral update:
        # 1. When stationary or drifting away from target, integrates clamped velocity error to learn hover trim
        #    and provide extra authority to unstick from barriers.
        # 2. Once moving toward target (position_error * current_vel > 0), never winds up further in the return
        #    direction and unwinds any return-direction integral proportional to distance traveled (-3.0 * v * dt)
        #    so temporary wall-unsticking integral sheds cleanly before reaching origin.
        if enable_integral:
            clamped_vel_err = float(np.clip(self.velocity_error, -self.max_integral_vel_error, self.max_integral_vel_error))
            if self.position_error * current_vel > 0.0:
                # Drone is actively moving toward target position:
                if self.vel_pid.integral * self.position_error > 0.0:
                    # Unwind return-direction (wall-unsticking) integral proportional to distance traveled toward origin
                    prev_sign = 1.0 if self.vel_pid.integral > 0.0 else -1.0
                    self.vel_pid.integral -= 3.0 * current_vel * dt
                    if self.vel_pid.integral * prev_sign < 0.0:
                        self.vel_pid.integral = 0.0
                elif self.position_error * self.velocity_error > 0.0:
                    # Moving toward target slower than target_velocity while integral opposes return: gently relax opposing integral
                    self.vel_pid.integral += clamped_vel_err * dt
            else:
                if abs(current_vel) > self.max_velocity and self.position_error * current_vel < 0.0:
                    # High-speed drift/overshoot away from target: unwind wrong-sign integral fast, accumulate right-sign trim at 0.75x
                    if self.vel_pid.integral * self.velocity_error < 0.0:
                        self.vel_pid.integral += 3.0 * clamped_vel_err * dt
                    else:
                        self.vel_pid.integral += 0.75 * clamped_vel_err * dt
                else:
                    # Normal hover / low-speed drift / stalled against barrier: integrate at full clamped rate to learn trim
                    self.vel_pid.integral += clamped_vel_err * dt
            self.vel_pid.integral = max(self.vel_pid.integral_min, min(self.vel_pid.integral_limit, self.vel_pid.integral))

        # Inner Loop: Velocity P+D + managed I control
        vel_pid_out = self.vel_pid.compute(current_vel, dt, enable_integral=False)
        return vel_pid_out
