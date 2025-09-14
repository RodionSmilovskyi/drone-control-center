class PIDController:
    """A simple PID controller."""

    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.last_error, self.integral = 0, 0

    def update_gains(self, Kp, Ki, Kd):
        """Allows for real-time updating of PID gains."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def compute(self, measurement, dt):
        """Get current value"""
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
    
    
    def reset(self):
        """
        Resets the controller's internal state. This should be called
        every time the environment is reset.
        """
        self.last_error = 0
        self.integral = 0
