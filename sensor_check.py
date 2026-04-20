# sensor_check_2_v2.py
# This script tests the PMW3901 (SPI) and *one* VL53L1X (I2C)
#
# v2: Fixed the 'get_address' bug. We just set the address and trust it.

import time
import board
import digitalio
import adafruit_vl53l1x
import pmw3901

# --- Configuration ---
# I2C (Range Sensor)
# Connect your *down-facing* sensor's XSHUT pin to GPIO 17
DOWN_SENSOR_SHUT_PIN = board.D17
DOWN_SENSOR_ADDRESS = 0x30        # New address we will assign

# SPI (Optical Flow)
# Connect your sensor's CS pin to BCM 8 (Physical Pin 24)
SPI_CS_PIN = 8

def initialize_sensors():
    """
    Attempts to initialize one I2C sensor and one SPI sensor.
    Returns (sensor_down, flow)
    """
    
    # --- 1. Check I2C Bus and Range Sensor ---
    print("--- Checking I2C Bus (VL53L1X)...")
    sensor_down = None
    
    try:
        # Create the I2C bus
        i2c = board.I2C()
        print("I2C bus OK.")

        # Create shutdown pin object
        xshut_down = digitalio.DigitalInOut(DOWN_SENSOR_SHUT_PIN)
        xshut_down.direction = digitalio.Direction.OUTPUT

        # Shut down sensor
        xshut_down.value = False
        time.sleep(0.1)

        # --- Initialize Down-Facing Sensor ---
        print(f"Bringing Down-Sensor (on GPIO {DOWN_SENSOR_SHUT_PIN}) online...")
        xshut_down.value = True
        time.sleep(0.1)
        
        # This line will fail if the sensor is not found at 0x29
        sensor_down = adafruit_vl53l1x.VL53L1X(i2c)
        
        print(f"Changing Down-Sensor address to {hex(DOWN_SENSOR_ADDRESS)}...")
        # This is the important line:
        sensor_down.set_address(DOWN_SENSOR_ADDRESS)
        # We just assume it worked. There is no 'get_address' to check.
        
        sensor_down.start_ranging()
        
        print("--- VL53L1X Sensor: SUCCESS! ---")

    except Exception as e:
        print(f"FAILED to initialize I2C sensor: {e}")
        print("Check I2C wiring (SDA, SCL) and ensure I2C is enabled.")
        print(f"Check XSHUT pin wiring (GPIO {DOWN_SENSOR_SHUT_PIN}).")

    # --- 2. Check Optical Flow (PMW3901) ---
    print("\n--- Checking Optical Flow (PMW3901)...")
    flow = None
    try:
        # This is the initialization that works with 'dtoverlay=spi0-0cs'
        flow = pmw3901.PMW3901(spi_cs_gpio=SPI_CS_PIN)
        flow.set_rotation(0)
        
        # Try to read the ID to confirm communication
        chip_id, revision = flow.get_id()
        print(f"Optical Flow (PMW3901) initialized SUCCESSFULLY. (ID: {hex(chip_id)}, Rev: {hex(revision)})")
        print("--- PMW3901 Sensor: SUCCESS! ---")
        
    except Exception as e:
        print(f"FAILED to initialize Optical Flow (PMW3901): {e}")
        print("This is likely the 'Timed out' error.")
        print("Please lift sensor > 8cm off a textured surface and move it.")
        flow = None # Mark as failed
        
    return sensor_down, flow

if __name__ == "__main__":
    sensor_down, flow = initialize_sensors()
    
    print("\n--- Sensor check complete! ---")
    
    if sensor_down and flow:
        print("\nReading from both sensors for 20 seconds...")
        print("(Move optical flow sensor to see values change)")
        start_time = time.time()
        
        # Keep track of the last known altitude
        current_altitude = "Waiting..." 
        
        while time.time() - start_time < 20:
            try:
                # 1. Read Optical Flow
                dx, dy = 0, 0 # Default to 0 instead of N/A for cleaner terminal output
                try:
                    dx, dy = flow.get_motion()
                except RuntimeError:
                    pass # Timed out, just skip
                
                # 2. Read Range Sensor and persist the value
                if sensor_down.data_ready:
                    current_altitude = f"{sensor_down.distance:.2f} cm"
                    sensor_down.clear_interrupt()
                
                # Print the persisted altitude and the fresh optical flow
                print(f"  Altitude: {current_altitude.ljust(12)} | Flow: X={dx:3}, Y={dy:3}")

            except Exception as e:
                print(f"  Read error: {e}")
                
            # Run at 100Hz!
            time.sleep(0.01)
        
    else:
        print("\nOne or more sensors failed to initialize. Please check errors above.")