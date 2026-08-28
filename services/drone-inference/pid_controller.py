class PIDController:
    """PID Controller with Derivative-on-Measurement to prevent setpoint kicks."""
    def __init__(self, Kp: float, Ki: float, Kd: float, setpoint: float = 0.0, integral_limit: float = 2.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.last_measurement = None
        self.integral_limit = integral_limit

    def reset(self):
        self.last_measurement = None
        self.integral = 0.0

    def compute(self, measurement: float, dt: float, enable_integral: bool = True) -> float:
        error = self.setpoint - measurement
        if enable_integral:
            self.integral += error * dt
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        else:
            self.integral = 0.0
        
        if self.last_measurement is not None and dt > 0:
            derivative = -(measurement - self.last_measurement) / dt
        else:
            derivative = 0.0
            
        self.last_measurement = measurement
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
