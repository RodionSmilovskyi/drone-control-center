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
        kp_pos: float = 0.35,
        max_velocity: float = 0.05,
        kp_vel: float = 0.65,
        ki_vel: float = 0.15,
        kd_vel: float = 0.20,
        integral_limit: float = 0.35,
        integral_min: float = -0.35,
        d_alpha: float = 0.35,
        max_integral_vel_error: float = 0.05,
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
        
        # Bounded, return-aware trim integral update (prevents high-velocity transient swings from corrupting hover trim)
        if enable_integral:
            clamped_vel_err = float(np.clip(self.velocity_error, -self.max_integral_vel_error, self.max_integral_vel_error))
            if self.position_error * self.velocity_error < 0.0:
                # Drone is already moving toward target faster than target_velocity (pre-crossing braking phase):
                if self.vel_pid.integral * self.velocity_error < 0.0:
                    # Integral was pushing toward target; unwind 2.0x faster so it doesn't cause zero-crossing overshoot
                    self.vel_pid.integral += 2.0 * clamped_vel_err * dt
                # Do not wind up integral in the opposing (braking) direction during a fast return
            else:
                if abs(current_vel) > self.max_velocity and self.position_error * current_vel < 0.0:
                    # High-speed overshoot away from target: unwind wrong-sign integral fast, accumulate right-sign gently
                    if self.vel_pid.integral * self.velocity_error < 0.0:
                        self.vel_pid.integral += 2.5 * clamped_vel_err * dt
                    else:
                        self.vel_pid.integral += 0.35 * clamped_vel_err * dt
                else:
                    # Normal hover / low-speed drift / stalled against barrier: integrate at full clamped rate to learn trim
                    self.vel_pid.integral += clamped_vel_err * dt
            self.vel_pid.integral = max(self.vel_pid.integral_min, min(self.vel_pid.integral_limit, self.vel_pid.integral))

        # Inner Loop: Velocity P+D + managed I control
        vel_pid_out = self.vel_pid.compute(current_vel, dt, enable_integral=False)
        return vel_pid_out
