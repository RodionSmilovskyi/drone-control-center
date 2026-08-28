import time
import math
import random

class SensorMock:
    def __init__(self):
        self.start_time = time.time()

    def read(self):
        """
        Generates fake data similar to what sensor_real produces.
        [altitude (0..3), front_dist (0..3), shift_x (-1..1), shift_y (-1..1), velocity_x (-1..1), velocity_y (-1..1)]
        """
        t = time.time() - self.start_time
        
        # Simulate some oscillatory movement
        altitude = ((math.sin(t * 0.5) + 1) / 2.0) * 3.0  # 0 to 3m
        front_dist = ((math.cos(t * 0.4) + 1.0) / 2.0) * 3.0  # 0 to 3m
        shift_x = math.sin(t * 1.2)              # -1 to 1
        shift_y = math.cos(t * 1.1)              # -1 to 1
        velocity_x = math.cos(t * 1.2) * 1.2     # -1.2 to 1.2 (clamped below)
        velocity_y = -math.sin(t * 1.1) * 1.1    # -1.1 to 1.1 (clamped below)
        
        # Add a bit of noise
        noise = lambda: random.uniform(-0.02, 0.02)

        return [
            max(0.0, min(3.0, altitude + noise())),
            max(0.0, min(3.0, front_dist + noise())),
            max(-1.0, min(1.0, shift_x + noise())),
            max(-1.0, min(1.0, shift_y + noise())),
            max(-1.0, min(1.0, velocity_x + noise())),
            max(-1.0, min(1.0, velocity_y + noise()))
        ]
