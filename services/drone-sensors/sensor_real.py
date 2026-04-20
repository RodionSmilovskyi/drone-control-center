import time
import board
import digitalio
import adafruit_vl53l1x
import pmw3901
import logging
import sys
import os
import threading
import math

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from drone_logging import setup_logger
try:
    from strategic_agent import MAX_XY_SHIFT
except ImportError:
    MAX_XY_SHIFT = 1.0

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
        
        # State variables
        self.altitude = 0.0
        self.shift_x = 0.0
        self.shift_y = 0.0
        self.vel_x_norm = 0.0
        self.vel_y_norm = 0.0
        self.last_time = time.time()

        # Optical Flow Constants (PMW3901)
        self.FOV_DEGREES = 42.0
        self.RESOLUTION = 35.0
        # precompute the scalar: (2 * tan(FOV/2)) / RESOLUTION
        self.FLOW_METERS_PER_PIXEL_PER_METER = (2 * math.tan(math.radians(self.FOV_DEGREES) / 2)) / self.RESOLUTION
        
        # Normalization and Calibration
        self.MAX_VELOCITY = 5.0
        self.MAX_XY_SHIFT = MAX_XY_SHIFT # From strategic_agent
        self.MAX_ALTITUDE = 1.0
        
        # Deadbands and Smoothing
        self.FLOW_DEADBAND = 0.05 # Threshold on filtered value to suppress jitter
        self.ALPHA_FLOW = 0.2    # Slightly faster response
        self.ALPHA_ALT = 0.3
        self.ALPHA_VEL = 0.2

        self.filtered_dx = 0.0
        self.filtered_dy = 0.0

        self.lock = threading.Lock()
        self._init_hardware()
        
        # Start background polling thread
        self.stop_thread = False
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

    def _init_hardware(self):
        try:
            i2c = board.I2C()
            xshut_down = digitalio.DigitalInOut(DOWN_SENSOR_SHUT_PIN)
            xshut_down.direction = digitalio.Direction.OUTPUT
            
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
            self.logger.critical(f"Hardware Initialization Error: {e}")

    def _poll_loop(self):
        """Background loop to poll sensors at high frequency."""
        warmup_end = time.time() + 5.0
        
        while not self.stop_thread:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            with self.lock:
                new_alt = self.altitude

            # 1. Altitude Reading
            raw_alt_m = new_alt
            if self.sensor_down:
                try:
                    if self.sensor_down.data_ready:
                        raw_dist = self.sensor_down.distance 
                        if raw_dist is not None:
                            raw_alt_m = max(0.0, min(self.MAX_ALTITUDE, raw_dist / 100.0))
                            new_alt = (self.ALPHA_ALT * raw_alt_m) + ((1.0 - self.ALPHA_ALT) * new_alt)
                        self.sensor_down.clear_interrupt()
                except Exception:
                    pass

            # 2. Optical Flow Reading
            if self.flow:
                try:
                    motion = self.flow.get_motion()
                    if motion is not None and current_time > warmup_end:
                        dx, dy = motion
                        
                        # 2a. Low Pass Filter raw motion
                        self.filtered_dx = (self.ALPHA_FLOW * dx) + (1.0 - self.ALPHA_FLOW) * self.filtered_dx
                        self.filtered_dy = (self.ALPHA_FLOW * dy) + (1.0 - self.ALPHA_FLOW) * self.filtered_dy

                        # 2b. Apply Deadbands to filtered motion
                        f_dx = self.filtered_dx if abs(self.filtered_dx) > self.FLOW_DEADBAND else 0.0
                        f_dy = self.filtered_dy if abs(self.filtered_dy) > self.FLOW_DEADBAND else 0.0
                        
                        integration_alt = max(0.02, raw_alt_m)

                        if integration_alt > 0.04: 
                            # distance in meters = counts * alt * (2*tan(FOV/2)/RES)
                            d_shift_x_m = f_dx * integration_alt * self.FLOW_METERS_PER_PIXEL_PER_METER
                            d_shift_y_m = f_dy * integration_alt * self.FLOW_METERS_PER_PIXEL_PER_METER
                            
                            with self.lock:
                                # Store in METERS
                                self.shift_x += d_shift_x_m
                                self.shift_y += d_shift_y_m
                                
                                # Hard Clamp shift in meters to [-MAX_XY_SHIFT, MAX_XY_SHIFT]
                                self.shift_x = max(-self.MAX_XY_SHIFT, min(self.MAX_XY_SHIFT, self.shift_x))
                                self.shift_y = max(-self.MAX_XY_SHIFT, min(self.MAX_XY_SHIFT, self.shift_y))
                                
                                if dt > 0:
                                    # Velocity normalized to MAX_VELOCITY
                                    raw_vx = max(-1.0, min(1.0, (d_shift_x_m / dt) / self.MAX_VELOCITY))
                                    raw_vy = max(-1.0, min(1.0, (d_shift_y_m / dt) / self.MAX_VELOCITY))
                                    self.vel_x_norm = (self.ALPHA_VEL * raw_vx) + ((1.0 - self.ALPHA_VEL) * self.vel_x_norm)
                                    self.vel_y_norm = (self.ALPHA_VEL * raw_vy) + ((1.0 - self.ALPHA_VEL) * self.vel_y_norm)
                        else:
                            with self.lock:
                                self.vel_x_norm = 0.0
                                self.vel_y_norm = 0.0
                                if raw_alt_m < 0.04:
                                    self.shift_x = 0.0
                                    self.shift_y = 0.0
                except (RuntimeError, Exception):
                    with self.lock:
                        self.vel_x_norm = 0.0
                        self.vel_y_norm = 0.0

            with self.lock:
                self.altitude = new_alt

            time.sleep(0.01)

    def reset_shifts(self):
        with self.lock:
            self.shift_x = 0.0
            self.shift_y = 0.0

    def read(self):
        with self.lock:
            return [self.altitude, self.shift_x, self.shift_y, self.vel_x_norm, self.vel_y_norm]
