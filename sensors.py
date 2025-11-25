import time
import json
import board
import digitalio
import adafruit_vl53l1x
import pmw3901
import paho.mqtt.client as mqtt
import logging
from drone_logging import setup_logger

# --- Configuration ---
MQTT_BROKER = "localhost" # Assuming running on the Pi itself
MQTT_PORT = 1883
SENSOR_TOPIC = "drone/sensors"
LOG_FILE = "sensor.log"
LOOP_FREQUENCY = 30 # Target 30Hz (fast enough for flow)

# --- Pin Definitions ---
# I2C (Range Sensors)
DOWN_SENSOR_SHUT_PIN = board.D17      # GPIO 17
OBSTACLE_SENSOR_SHUT_PIN = board.D27  # GPIO 27
DOWN_SENSOR_ADDRESS = 0x30            # New address for down sensor
OBSTACLE_SENSOR_ADDRESS = 0x29        # Default address for obstacle sensor

# SPI (Optical Flow)
# Use the standard GPIO pin for CS (BCM 8 / CE0)
# Ensure 'dtoverlay=spi0-0cs' is in /boot/firmware/config.txt so kernel doesn't fight for it
SPI_CS_PIN = 8 

# --- Setup Logger ---
logger = setup_logger("Sensor_Node", LOG_FILE)

# --- Initialization Functions ---
def initialize_hardware():
    """Initializes all sensors with the robust reset logic."""
    sensor_down = None
    sensor_obstacle = None
    flow = None
    
    try:
        # --- 1. I2C Setup ---
        i2c = board.I2C()
        
        # Shutdown pins
        xshut_down = digitalio.DigitalInOut(DOWN_SENSOR_SHUT_PIN)
        xshut_down.direction = digitalio.Direction.OUTPUT
        xshut_obstacle = digitalio.DigitalInOut(OBSTACLE_SENSOR_SHUT_PIN)
        xshut_obstacle.direction = digitalio.Direction.OUTPUT
        
        # Reset sequence
        logger.info("Resetting I2C sensors...")
        xshut_down.value = False
        xshut_obstacle.value = False
        time.sleep(0.1)
        
        # Init Down Sensor
        xshut_down.value = True
        time.sleep(0.1)
        try:
            sensor_down = adafruit_vl53l1x.VL53L1X(i2c)
            sensor_down.set_address(DOWN_SENSOR_ADDRESS)
            sensor_down.start_ranging()
            logger.info(f"Down-Sensor initialized at {hex(DOWN_SENSOR_ADDRESS)}")
        except Exception as e:
            logger.error(f"Failed to init Down-Sensor: {e}")

        # Init Obstacle Sensor
        # xshut_obstacle.value = True
        # time.sleep(0.1)
        # try:
        #     sensor_obstacle = adafruit_vl53l1x.VL53L1X(i2c) # Default 0x29
        #     sensor_obstacle.start_ranging()
        #     logger.info(f"Obstacle-Sensor initialized at {hex(OBSTACLE_SENSOR_ADDRESS)}")
        # except Exception as e:
        #     logger.error(f"Failed to init Obstacle-Sensor: {e}")

        # --- 2. Optical Flow Setup ---
        try:
            flow = pmw3901.PMW3901(spi_cs_gpio=SPI_CS_PIN)
            flow.set_rotation(0)
            # Quick read to verify
            chip_id, rev = flow.get_id()
            logger.info(f"Optical Flow initialized. ID: {hex(chip_id)}")
        except Exception as e:
            logger.error(f"Failed to init Optical Flow: {e}")

    except Exception as e:
        logger.critical(f"Hardware Initialization Error: {e}")

    return sensor_down, sensor_obstacle, flow

def main():
    # 1. Init Hardware
    sensor_down, sensor_obstacle, flow = initialize_hardware()
    
    if not any([sensor_down, sensor_obstacle, flow]):
        logger.critical("No sensors initialized. Exiting.")
        return

    # 2. Init MQTT
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="pi_sensor_node")
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        logger.info("Connected to MQTT.")
    except Exception as e:
        logger.critical(f"MQTT Connection failed: {e}")
        return

    # 3. Main Loop
    logger.info("Starting Sensor Loop...")
    loop_time = 1.0 / LOOP_FREQUENCY
    
    try:
        while True:
            loop_start = time.time()
            payload = {}

            try:
                # --- Read Down Sensor (Altitude) ---
                if sensor_down and sensor_down.data_ready:
                    # Convert cm to meters for consistency with FC data
                    payload['altitude'] = round(sensor_down.distance / 100.0, 3) 
                    sensor_down.clear_interrupt()

                # --- Read Obstacle Sensor ---
                if sensor_obstacle and sensor_obstacle.data_ready:
                    payload['obstacle_distance'] = round(sensor_obstacle.distance / 100.0, 3)
                    sensor_obstacle.clear_interrupt()

                # --- Read Optical Flow ---
                if flow:
                    try:
                        dx, dy = flow.get_motion()
                        # Only send if there is motion or periodically? 
                        # Sending zeros is safer for the agent to know "no motion" vs "no data"
                        payload['flow'] = {'x': dx, 'y': dy}
                    except RuntimeError:
                        pass # Sensor read timed out (normal if no motion/texture)

                # --- Publish ---
                if payload:
                    # Add timestamp to help agent sync data
                    payload['timestamp'] = time.time()
                    client.publish(SENSOR_TOPIC, json.dumps(payload))
                    # logger.debug(f"Pub: {payload}") # Uncomment for verbose debug

            except Exception as e:
                logger.error(f"Error in main loop: {e}")

            # --- Timing ---
            elapsed = time.time() - loop_start
            if elapsed < loop_time:
                time.sleep(loop_time - elapsed)

    except KeyboardInterrupt:
        logger.info("Stopping Sensor Node...")

    # Cleanup (Now reachable!)
    if sensor_down: sensor_down.stop_ranging()
    if sensor_obstacle: sensor_obstacle.stop_ranging()
    client.loop_stop()
    logger.info("Cleanup complete.")

if __name__ == "__main__":
    logger.setLevel(logging.INFO)
    main()