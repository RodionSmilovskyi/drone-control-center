class PIDController:
    """PID Controller with Derivative-on-Measurement to prevent setpoint kicks."""
    def __init__(self, Kp: float, Ki: float, Kd: float, setpoint: float = 0.0, integral_limit: float = 2.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.last_measurement = None
        self.integral_limit = integral_limit
        self.derivative = 0.0
        self.time_since_last_change = 0.0

    def reset(self):
        self.last_measurement = None
        self.integral = 0.0
        self.derivative = 0.0
        self.time_since_last_change = 0.0

    def compute(self, measurement: float, dt: float, enable_integral: bool = True) -> float:
        error = self.setpoint - measurement
        if enable_integral:
            self.integral += error * dt
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        if self.last_measurement is not None:
            if measurement != self.last_measurement:
                real_dt = max(self.time_since_last_change + dt, 0.001)
                new_derivative = -(measurement - self.last_measurement) / real_dt
                self.derivative = 0.6 * new_derivative + 0.4 * self.derivative
                self.last_measurement = measurement
                self.time_since_last_change = 0.0
            else:
                self.time_since_last_change += dt
                self.derivative *= 0.95
        else:
            self.derivative = 0.0
            self.last_measurement = measurement
            self.time_since_last_change = 0.0
            
        return self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative
