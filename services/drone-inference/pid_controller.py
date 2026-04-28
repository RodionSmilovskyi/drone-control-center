class PIDController:
    """PID Controller with Derivative-on-Measurement to prevent setpoint kicks."""
    def __init__(self, Kp: float, Ki: float, Kd: float, setpoint: float = 0.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.last_measurement = 0.0

    def reset(self):
        self.last_measurement = 0.0
        self.integral = 0.0

    def compute(self, measurement: float, dt: float) -> float:
        error = self.setpoint - measurement
        self.integral += error * dt
        if dt > 0:
            derivative = -(measurement - self.last_measurement) / dt
        else:
            derivative = 0.0
        self.last_measurement = measurement
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
