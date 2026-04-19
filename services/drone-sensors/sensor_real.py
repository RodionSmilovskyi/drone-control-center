import time
import board
import digitalio
import adafruit_vl53l1x
import pmw3901
import logging
import sys
import os

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from drone_logging import setup_logger

# --- Configuration ---
LOG_FILE = "sensor.log"

# --- Pin Definitions ---
DOWN_SENSOR_SHUT_PIN = board.D17
OBSTACLE_SENSOR_SHUT_PIN = board.D27
DOWN_SENSOR_ADDRESS = 0x30
SPI_CS_PIN = 8 

class SensorReal:
    def __init__(self):
        self.logger = setup_logger("Sensor_Real", LOG_FILE, logging.INFO)
        self.sensor_down = None
        self.flow = None
        self.altitude = 0.0
        self.shift_x = 0.0
        self.shift_y = 0.0
        self.last_time = time.time()
        
        # Normalization constants (matching strategic_agent.py)
        self.FLOW_SCALAR = 0.14
        self.MAX_VELOCITY = 5.0
        self.MAX_XY_SHIFT = 1.0
        self.MAX_ALTITUDE = 1.0
        
        self._init_hardware()

    def _init_hardware(self):
        try:
            i2c = board.I2C()
            xshut_down = digitalio.DigitalInOut(DOWN_SENSOR_SHUT_PIN)
            xshut_down.direction = digitalio.Direction.OUTPUT
            
            # Reset sequence
            xshut_down.value = False
            time.sleep(0.1)
            xshut_down.value = True
            time.sleep(0.1)
            
            try:
                self.sensor_down = adafruit_vl53l1x.VL53L1X(i2c)
                self.sensor_down.set_address(DOWN_SENSOR_ADDRESS)
                self.sensor_down.start_ranging()
                self.logger.info("Down-Sensor initialized.")
            except Exception as e:
                self.logger.error(f"Failed to init Down-Sensor: {e}")

            try:
                self.flow = pmw3901.PMW3901(spi_cs_gpio=SPI_CS_PIN)
                self.flow.set_rotation(0)
                self.logger.info("Optical Flow initialized.")
            except Exception as e:
                self.logger.error(f"Failed to init Optical Flow: {e}")

        except Exception as e:
            def read(self):
                """
                Returns [altitude (0..1), shift_x (-1..1), shift_y (-1..1), velocity_x (-1..1), velocity_y (-1..1)]
                """
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time

                # 1. Altitude Reading (VL53L1X)
                if self.sensor_down:
                    try:
                        if self.sensor_down.data_ready:
                            # Sensor returns cm. Map 0-100cm to 0.0-1.0m
                            raw_dist = self.sensor_down.distance
                            if raw_dist is not None:
                                self.altitude = max(0.0, min(self.MAX_ALTITUDE, raw_dist / 100.0))
                            self.sensor_down.clear_interrupt()
                    except Exception as e:
                        self.logger.error(f"Error reading altitude: {e}")

                # 2. Optical Flow Reading (PMW3901)
                vel_x_norm, vel_y_norm = 0.0, 0.0
                if self.flow:
                    try:
                        # get_motion can block or throw RuntimeError on timeout
                        # We use a try/except to catch timeouts immediately
                        motion = self.flow.get_motion()
                        if motion is not None:
                            dx, dy = motion

                            # Physics calculation as done in strategic_agent.py:
                            d_shift_x = dx * self.altitude * self.FLOW_SCALAR
                            d_shift_y = dy * self.altitude * self.FLOW_SCALAR

                            self.shift_x = max(-self.MAX_XY_SHIFT, min(self.MAX_XY_SHIFT, self.shift_x + d_shift_x))
                            self.shift_y = max(-self.MAX_XY_SHIFT, min(self.MAX_XY_SHIFT, self.shift_y + d_shift_y))

                            if dt > 0:
                                vx_phys = d_shift_x / dt
                                vy_phys = d_shift_y / dt
                                # Normalize by MAX_VELOCITY to match SensorMock range [-1, 1]
                                vel_x_norm = max(-1.0, min(1.0, vx_phys / self.MAX_VELOCITY))
                                vel_y_norm = max(-1.0, min(1.0, vy_phys / self.MAX_VELOCITY))
                    except (RuntimeError, Exception):
                        # If flow fails (common when lifted or out of range), 
                        # we just report zero velocity for this frame to avoid blocking
                        pass

                return [self.altitude, self.shift_x, self.shift_y, vel_x_norm, vel_y_norm]
