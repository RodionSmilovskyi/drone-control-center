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
            self.logger.critical(f"Hardware Initialization Error: {e}")

    def read(self):
        """
        Returns [altitude, shift_x, shift_y, velocity_x, velocity_y]
        Note: Real sensor mapping to this format needs calibration.
        """
        altitude = 0.0
        shift_x, shift_y = 0.0, 0.0
        vel_x, vel_y = 0.0, 0.0

        if self.sensor_down and self.sensor_down.data_ready:
            altitude = self.sensor_down.distance / 100.0
            self.sensor_down.clear_interrupt()

        if self.flow:
            try:
                dx, dy = self.flow.get_motion()
                # Dummy mapping for now: dx/dy as velocities
                vel_x, vel_y = float(dx), float(dy)
            except RuntimeError:
                pass

        return [altitude, shift_x, shift_y, vel_x, vel_y]
