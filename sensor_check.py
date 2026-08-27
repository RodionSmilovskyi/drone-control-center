# sensor_check.py
# This script tests the PMW3901 (SPI) and two VL53L1X range sensors (I2C: down & front)
#
# v2: Fixed the 'get_address' bug. We just set the address and trust it.

import time
import os
import sys
import board
import digitalio
import adafruit_vl53l1x
import pmw3901

# --- Configuration ---
LOG_FILE = "sensor_check.log"

def log_print(msg, log_f=None):
    """Prints message to stdout and appends to log file."""
    print(msg)
    if log_f:
        log_f.write(msg + "\n")
        log_f.flush()

# I2C (Range Sensor)
# Connect your *down-facing* sensor's XSHUT pin to GPIO 17
DOWN_SENSOR_SHUT_PIN = board.D17
DOWN_SENSOR_ADDRESS = 0x30        # New address we will assign

# Connect your *front-facing* sensor's XSHUT pin to GPIO 27
FRONT_SENSOR_SHUT_PIN = board.D27
FRONT_SENSOR_ADDRESS = 0x31

# SPI (Optical Flow)
# Connect your sensor's CS pin to BCM 8 (Physical Pin 24)
SPI_CS_PIN = 8

def initialize_sensors(log_f=None):
    """
    Attempts to initialize two I2C sensors and one SPI sensor.
    Returns (sensor_down, sensor_front, flow)
    """
    
    # --- 1. Check I2C Bus and Range Sensors ---
    log_print("--- Checking I2C Bus (VL53L1X)...", log_f)
    sensor_down = None
    sensor_front = None
    
    try:
        # Create the I2C bus
        i2c = board.I2C()
        log_print("I2C bus OK.", log_f)

        # Create shutdown pin objects
        xshut_down = digitalio.DigitalInOut(DOWN_SENSOR_SHUT_PIN)
        xshut_down.direction = digitalio.Direction.OUTPUT
        
        xshut_front = digitalio.DigitalInOut(FRONT_SENSOR_SHUT_PIN)
        xshut_front.direction = digitalio.Direction.OUTPUT

        # Shut down sensors
        xshut_down.value = False
        xshut_front.value = False
        time.sleep(0.1)

        # --- Initialize Down-Facing Sensor ---
        log_print(f"Bringing Down-Sensor (on GPIO {DOWN_SENSOR_SHUT_PIN}) online...", log_f)
        xshut_down.value = True
        time.sleep(0.1)
        
        # This line will fail if the sensor is not found at 0x29
        sensor_down = adafruit_vl53l1x.VL53L1X(i2c)
        
        log_print(f"Changing Down-Sensor address to {hex(DOWN_SENSOR_ADDRESS)}...", log_f)
        sensor_down.set_address(DOWN_SENSOR_ADDRESS)
        sensor_down.start_ranging()
        
        log_print("--- VL53L1X Down Sensor: SUCCESS! ---", log_f)

        # --- Initialize Front-Facing Sensor ---
        log_print(f"Bringing Front-Sensor (on GPIO {FRONT_SENSOR_SHUT_PIN}) online...", log_f)
        xshut_front.value = True
        time.sleep(0.1)
        
        sensor_front = adafruit_vl53l1x.VL53L1X(i2c)
        
        log_print(f"Changing Front-Sensor address to {hex(FRONT_SENSOR_ADDRESS)}...", log_f)
        sensor_front.set_address(FRONT_SENSOR_ADDRESS)
        sensor_front.start_ranging()
        
        log_print("--- VL53L1X Front Sensor: SUCCESS! ---", log_f)

    except Exception as e:
        log_print(f"FAILED to initialize I2C sensor: {e}", log_f)
        log_print("Check I2C wiring (SDA, SCL) and ensure I2C is enabled.", log_f)
        log_print(f"Check XSHUT pin wiring (Down: GPIO {DOWN_SENSOR_SHUT_PIN}, Front: GPIO {FRONT_SENSOR_SHUT_PIN}).", log_f)

    # --- 2. Check Optical Flow (PMW3901) ---
    log_print("\n--- Checking Optical Flow (PMW3901)...", log_f)
    flow = None
    try:
        # This is the initialization that works with 'dtoverlay=spi0-0cs'
        flow = pmw3901.PMW3901(spi_cs_gpio=SPI_CS_PIN)
        flow.set_rotation(0)
        
        # Try to read the ID to confirm communication
        chip_id, revision = flow.get_id()
        log_print(f"Optical Flow (PMW3901) initialized SUCCESSFULLY. (ID: {hex(chip_id)}, Rev: {hex(revision)})", log_f)
        log_print("--- PMW3901 Sensor: SUCCESS! ---", log_f)
        
    except Exception as e:
        log_print(f"FAILED to initialize Optical Flow (PMW3901): {e}", log_f)
        log_print("This is likely the 'Timed out' error.", log_f)
        log_print("Please lift sensor > 8cm off a textured surface and move it.", log_f)
        flow = None # Mark as failed
        
    return sensor_down, sensor_front, flow

if __name__ == "__main__":
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), LOG_FILE)
    log_f = open(log_path, "w")
    log_print(f"=== Sensor Diagnostic Check ({time.strftime('%Y-%m-%d %H:%M:%S')}) ===", log_f)
    
    try:
        sensor_down, sensor_front, flow = initialize_sensors(log_f)
        
        log_print("\n--- Sensor check complete! ---", log_f)
        
        if sensor_down and sensor_front and flow:
            log_print("\nReading from all sensors for 20 seconds...", log_f)
            log_print("Cumulative Tracking Started...", log_f)
            start_time = time.time()
            last_log_print = 0.0
            
            current_altitude = 0.0 # Keep as a float for math
            current_front_dist = 0.0
            cumulative_x = 0
            cumulative_y = 0
            
            while time.time() - start_time < 20:
                try:
                    # 1. Read Range Sensors
                    if sensor_down.data_ready:
                        current_altitude = sensor_down.distance # Distance in cm
                        sensor_down.clear_interrupt()
                    
                    if sensor_front.data_ready:
                        current_front_dist = sensor_front.distance
                        sensor_front.clear_interrupt()
                    
                    # 2. Read Optical Flow
                    try:
                        dx, dy = flow.get_motion()
                        # Add to the running total
                        cumulative_x += dx
                        cumulative_y += dy
                    except RuntimeError:
                        pass 
                    
                    # Print and log periodic readings (every ~200ms to avoid bloated logs while remaining readable)
                    now = time.time()
                    if now - last_log_print >= 0.2:
                        log_print(f"  Alt: {current_altitude:5.1f} cm | Front: {current_front_dist:5.1f} cm | Pos (pixels): X={cumulative_x:4}, Y={cumulative_y:4}", log_f)
                        last_log_print = now

                except Exception as e:
                    pass
                    
                time.sleep(0.01)
            
            log_print("\nSensor diagnostics completed successfully.", log_f)
        else:
            log_print("\nOne or more sensors failed to initialize. Please check errors above.", log_f)
    finally:
        log_print(f"Diagnostic log saved to: {log_path}", log_f)
        log_f.close()