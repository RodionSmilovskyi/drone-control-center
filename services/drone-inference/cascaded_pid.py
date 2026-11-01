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
        kp_pos: float = 0.25,
        max_velocity: float = 0.06,
        kp_vel: float = 0.55,
        ki_vel: float = 0.30,
        kd_vel: float = 0.18,
        integral_limit: float = 0.35,
        integral_min: float = -0.35,
        d_alpha: float = 0.35,
        max_integral_vel_error: float = 0.06,
        max_d_accel: float = 0.80,
    ):
        self.kp_pos = kp_pos
        self.max_velocity = max_velocity
        self.max_integral_vel_error = max_integral_vel_error
        self.max_d_accel = max_d_accel
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
            max_derivative=max_d_accel,
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
        
        # Anti-slingshot, stationary-gated trim & wall-unsticking integral update:
        # 1. Separate static trim/wall-stall from dynamic swing momentum:
        #    - When nearly stationary (|current_vel| <= 0.04 m/s), velocity error reflects true hover bias
        #      or wall stall (with a 2.5x unsticking boost when displaced > 8cm).
        #    - When actively moving (|current_vel| > 0.04 m/s), unwind any wrong-sign integral that is
        #      propelling the overshoot (3.0x), and do NOT wind up on transient swing momentum away from target.
        # 2. When moving toward target faster than target_velocity (position_error * velocity_error < 0),
        #    freeze opposing (braking) windup and briskly unwind return-direction integral toward 0 so P-braking
        #    is never overpowered by stale unsticking integral.
        if enable_integral:
            clamped_vel_err = float(np.clip(self.velocity_error, -self.max_integral_vel_error, self.max_integral_vel_error))
            if self.position_error * self.velocity_error >= 0.0:
                if self.vel_pid.integral * self.velocity_error < 0.0:
                    # Integral has the wrong sign (actively pushing against velocity_error): unwind fast
                    self.vel_pid.integral += 3.0 * clamped_vel_err * dt
                elif abs(current_vel) <= 0.04 and abs(self.vel_pid.derivative) <= 0.20:
                    # Stationary / low-speed hover or genuine wall stall (|v| <= 4 cm/s and low acceleration |a| <= 0.20 m/s^2,
                    # excluding dynamic swing turnaround peaks where v briefly crosses 0 under high acceleration):
                    if abs(self.position_error) > 0.08 and abs(current_vel) <= 0.03:
                        unstick_step = float(np.sign(clamped_vel_err)) * max(abs(clamped_vel_err), 0.05)
                        self.vel_pid.integral += 4.0 * unstick_step * dt
                    else:
                        self.vel_pid.integral += clamped_vel_err * dt
                elif self.position_error * current_vel >= 0.0 and abs(self.vel_pid.derivative) <= 0.20:
                    # Creeping slowly toward target (between 4 cm/s and target_velocity): gentle trim update
                    self.vel_pid.integral += 0.15 * clamped_vel_err * dt
                # Else (abs(current_vel) > 0.04 and moving away from target): freeze integral to prevent swing slingshot
            else:
                # Moving toward target faster than target_velocity (position_error * velocity_error < 0):
                # Never wind up in the opposing (braking) direction; briskly relax return-direction integral toward 0.
                if self.vel_pid.integral * self.position_error > 0.0:
                    prev_sign = 1.0 if self.vel_pid.integral > 0.0 else -1.0
                    unwind_gain = 3.0 if abs(current_vel) > 1.5 * self.max_velocity else 1.2
                    self.vel_pid.integral += unwind_gain * clamped_vel_err * dt
                    if self.vel_pid.integral * prev_sign < 0.0:
                        self.vel_pid.integral = 0.0
            self.vel_pid.integral = max(self.vel_pid.integral_min, min(self.vel_pid.integral_limit, self.vel_pid.integral))

        # Inner Loop: Velocity P+D + managed I control
        vel_pid_out = self.vel_pid.compute(current_vel, dt, enable_integral=False)
        return vel_pid_out
