import time
import board
import digitalio
import adafruit_vl53l1x
import pmw3901
import logging
import sys
import os
import threading

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
        
        # State variables
        self.altitude = 0.0
        self.shift_x = 0.0
        self.shift_y = 0.0
        self.vel_x_norm = 0.0
        self.vel_y_norm = 0.0
        self.last_time = time.time()
        
        # Normalization and Calibration
        self.FLOW_SCALAR = 0.094
        self.MAX_VELOCITY = 5.0
        self.MAX_XY_SHIFT = 1.0
        self.MAX_ALTITUDE = 1.0
        self.FLOW_DEADBAND = 0 # Raw data for calibration
        
        # Smoothing (Exponential Moving Average)
        self.ALPHA_ALT = 0.3 # Smoothing for altitude
        self.ALPHA_VEL = 0.2 # Heavier smoothing for velocity
        
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
            self.logger.critical(f"Hardware Initialization Error: {e}")

    def _poll_loop(self):
        """Background loop to poll sensors at high frequency."""
        warmup_end = time.time() + 2.0 # 2 seconds warmup for sensor stabilization
        while not self.stop_thread:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            with self.lock:
                new_alt = self.altitude

            new_vx_norm = 0.0
            new_vy_norm = 0.0
            
            # 1. Altitude Reading
            if self.sensor_down:
                try:
                    if self.sensor_down.data_ready:
                        raw_dist = self.sensor_down.distance
                        if raw_dist is not None:
                            target_alt = max(0.0, min(self.MAX_ALTITUDE, raw_dist / 100.0))
                            # Smooth Altitude
                            new_alt = (self.ALPHA_ALT * target_alt) + ((1.0 - self.ALPHA_ALT) * new_alt)
                        self.sensor_down.clear_interrupt()
                except Exception:
                    pass

            # 2. Optical Flow Reading
            if self.flow:
                try:
                    motion = self.flow.get_motion()
                    if motion is not None and current_time > warmup_end:
                        dx, dy = motion
                        
                        # Apply deadband
                        if abs(dx) <= self.FLOW_DEADBAND: dx = 0
                        if abs(dy) <= self.FLOW_DEADBAND: dy = 0

                        if new_alt > 0.05:
                            # Scalar is multiplied by altitude because flow magnitude increases with distance
                            d_shift_x = dx * new_alt * self.FLOW_SCALAR
                            d_shift_y = dy * new_alt * self.FLOW_SCALAR
                            
                            with self.lock:
                                # Pure integration for position (no EMA here to avoid lag/scaling issues)
                                self.shift_x += d_shift_x
                                self.shift_y += d_shift_y
                                
                                # Clamp shifts
                                self.shift_x = max(-self.MAX_XY_SHIFT, min(self.MAX_XY_SHIFT, self.shift_x))
                                self.shift_y = max(-self.MAX_XY_SHIFT, min(self.MAX_XY_SHIFT, self.shift_y))
                                
                                # Smooth Velocity
                                if dt > 0:
                                    raw_vx_norm = max(-1.0, min(1.0, (d_shift_x / dt) / self.MAX_VELOCITY))
                                    raw_vy_norm = max(-1.0, min(1.0, (d_shift_y / dt) / self.MAX_VELOCITY))
                                    self.vel_x_norm = (self.ALPHA_VEL * raw_vx_norm) + ((1.0 - self.ALPHA_VEL) * self.vel_x_norm)
                                    self.vel_y_norm = (self.ALPHA_VEL * raw_vy_norm) + ((1.0 - self.ALPHA_VEL) * self.vel_y_norm)
                        else:
                            # Too low to track or landed
                            with self.lock:
                                self.vel_x_norm = 0.0
                                self.vel_y_norm = 0.0
                                if new_alt < 0.04: # Reset shifts if definitively landed
                                    self.shift_x = 0.0
                                    self.shift_y = 0.0
                        
                        new_vx_norm = self.vel_x_norm
                        new_vy_norm = self.vel_y_norm
                except (RuntimeError, Exception):
                    # Tracking lost, keep previous shift but reset velocity
                    with self.lock:
                        self.vel_x_norm = 0.0
                        self.vel_y_norm = 0.0
                    new_vx_norm = 0.0
                    new_vy_norm = 0.0

            # Update shared state
            with self.lock:
                self.altitude = new_alt

            # Small sleep to yield, but poll fast enough for flow (up to 100Hz)
            time.sleep(0.01)

    def reset_shifts(self):
        """Resets the integrated horizontal shifts to zero."""
        with self.lock:
            self.shift_x = 0.0
            self.shift_y = 0.0

    def read(self):
        """Returns the latest sensor state from the background thread."""
        with self.lock:
            return [self.altitude, self.shift_x, self.shift_y, self.vel_x_norm, self.vel_y_norm]

