class PIDController:
    """PID Controller with Derivative-on-Measurement to prevent setpoint kicks."""
    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        setpoint: float = 0.0,
        integral_limit: float = 1.2,
        integral_min: float = -0.35,
        d_alpha: float = 0.6,
        anti_windup_return: bool = False,
    ):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.last_measurement = None
        self.integral_limit = integral_limit
        self.integral_min = integral_min
        self.d_alpha = d_alpha
        self.anti_windup_return = anti_windup_return
        self.derivative = 0.0
        self.time_since_last_change = 0.0

    def reset(self):
        self.last_measurement = None
        self.integral = 0.0
        self.derivative = 0.0
        self.time_since_last_change = 0.0

    def compute(self, measurement: float, dt: float, enable_integral: bool = True) -> float:
        error = self.setpoint - measurement

        if self.last_measurement is not None:
            if measurement != self.last_measurement:
                real_dt = max(self.time_since_last_change + dt, 0.001)
                new_derivative = -(measurement - self.last_measurement) / real_dt
                self.derivative = self.d_alpha * new_derivative + (1.0 - self.d_alpha) * self.derivative
                self.last_measurement = measurement
                self.time_since_last_change = 0.0
            else:
                self.time_since_last_change += dt
                self.derivative *= 0.95
        else:
            self.derivative = 0.0
            self.last_measurement = measurement
            self.time_since_last_change = 0.0

        if enable_integral:
            if self.anti_windup_return:
                if self.integral * error < 0.0:
                    # Crossed setpoint: unwind opposing integral 3.5x faster so old windup never pushes into a barrier
                    self.integral += 3.5 * error * dt
                elif error * self.derivative < 0.0 and abs(self.derivative) > 0.08:
                    # Already moving back toward setpoint (> 8 cm/s): hold/gently bleed integral to prevent return overshoot
                    self.integral *= max(0.0, 1.0 - 0.1 * dt)
                else:
                    # Drifting away or stalled/glued against a barrier (|v| <= 8 cm/s): integrate at full rate to unstick
                    self.integral += error * dt
            else:
                self.integral += error * dt
            self.integral = max(self.integral_min, min(self.integral_limit, self.integral))

        return self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative
