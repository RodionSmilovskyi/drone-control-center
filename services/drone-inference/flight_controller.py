import numpy as np
from pid_controller import PIDController

class FlightController:
    """Low-level controller translating high-level actions to RC commands."""
    def __init__(self):
        # Confined-space tuning paired with Betaflight vbat_sag_compensation:
        # Hover baseline 1535 PWM ensures prompt, gentle unweighting (~1570 PWM at t=0.6s) without catapulting.
        # Integral trim (+165 PWM headroom -> 1700 PWM ceiling) briskly elevates throttle for sagged batteries.
        self.throttle_pid = PIDController(Kp=5.2, Ki=1.8, Kd=2.2, integral_limit=1.2, integral_min=-0.35)
        # Overdamped horizontal PIDs (Kd/Kp ≈ 0.85, minimal Ki) prevent circular/toilet-bowl overshoot
        self.roll_pid = PIDController(Kp=0.65, Ki=0.02, Kd=0.55, integral_limit=0.15, integral_min=-0.15)
        self.pitch_pid = PIDController(Kp=0.65, Ki=0.02, Kd=0.55, integral_limit=0.15, integral_min=-0.15)
        self.yaw_pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.1, integral_limit=0.2, integral_min=-0.2)
        
        self.hover_throttle = 1535
        self.min_throttle = 1341
        self.max_throttle = 1800
        # Expanded descent authority [-150] allows throttle down to 1385 PWM so drone can brake and descend on fresh packs,
        # headroom [+165] allows throttle up to 1700 PWM for depleted batteries.
        self.max_climb_correction = 165.0
        self.max_descent_correction = -150.0
        self.max_pid_correction = self.max_climb_correction
        
        # Explicit degree-to-PWM indoor tilt authority limits (SpeedyBee max angle ~50 deg -> 500 PWM)
        self.fc_max_angle_deg = 50.0
        self.max_indoor_tilt_deg = 5.0
        self.max_rp_correction = (self.max_indoor_tilt_deg / self.fc_max_angle_deg) * 500.0
        self.max_yaw_correction = 50.0
        
        # Ground threshold: 0.045m (above resting 0.028-0.031m and spin-up vibration ~0.043m) to prevent ground windup
        self.ground_threshold_norm = 0.045 / 3.0
        
        # Slew rate: 0.35 m/s (~0.117 normalized units/sec) to smoothly lead climb without ceiling overshoot
        self.max_climb_rate_norm = 0.35 / 3.0
        self.current_setpoint_norm = None
        self.ref_alt_norm = None
        self.ref_shift_x_norm = None
        self.ref_shift_y_norm = None
        
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

        is_airborne = current_alt_norm >= self.ground_threshold_norm

        # Latch reference state when AI mode engages (first frame after reset)
        if self.ref_alt_norm is None:
            self.ref_alt_norm = current_alt_norm
        if self.ref_shift_x_norm is None:
            self.ref_shift_x_norm = current_shift_x_norm if is_airborne else 0.0
        if self.ref_shift_y_norm is None:
            self.ref_shift_y_norm = current_shift_y_norm if is_airborne else 0.0

        # On the ground before takeoff, horizontal origin is always zero
        if not is_airborne:
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
        
        # Anti-windup: accumulate integral once airborne past ground threshold.
        # Below setpoint: allow continuous integral accumulation so hover throttle adapts as battery sags.
        # Above setpoint: decay integral back towards 0 without negative windup.
        is_airborne = current_alt_norm >= self.ground_threshold_norm
        enable_integral = is_airborne
        throttle_pid_out = self.throttle_pid.compute(current_alt_norm, dt, enable_integral=enable_integral)
        
        # Asymmetrical PID bounds: tight descent authority [-75] to eliminate ground bounce,
        # expanded climb headroom [+130] for battery sag compensation (throttle up to ~1680 PWM)
        pid_correction = np.clip(100.0 * throttle_pid_out, self.max_descent_correction, self.max_climb_correction)
        rc_throttle = self.hover_throttle + pid_correction

        # Compute Roll, Pitch, and Yaw PIDs; keep neutral (1500 PWM) until airborne for level liftoff
        roll_pid_out = self.roll_pid.compute(current_shift_x_norm, dt, enable_integral=enable_integral)
        pitch_pid_out = self.pitch_pid.compute(current_shift_y_norm, dt, enable_integral=enable_integral)
        yaw_pid_out = self.yaw_pid.compute(current_yaw_norm, dt, enable_integral=enable_integral)

        if is_airborne:
            roll_correction = np.clip(100.0 * roll_pid_out, -self.max_rp_correction, self.max_rp_correction)
            pitch_correction = np.clip(100.0 * pitch_pid_out, -self.max_rp_correction, self.max_rp_correction)
            yaw_correction = np.clip(100.0 * yaw_pid_out, -self.max_yaw_correction, self.max_yaw_correction)
        else:
            roll_correction = 0.0
            pitch_correction = 0.0
            yaw_correction = 0.0

        rc_roll = 1500.0 + roll_correction
        rc_pitch = 1500.0 + pitch_correction
        rc_yaw = 1500.0 + yaw_correction

        rc_commands = np.clip([rc_throttle, rc_roll, rc_pitch, rc_yaw], 1000, 2000)
        rc_commands[0] = np.clip(rc_commands[0], self.min_throttle, self.max_throttle)
        
        return rc_commands.astype(np.float32)
