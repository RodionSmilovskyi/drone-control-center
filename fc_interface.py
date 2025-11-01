import time
import json
import paho.mqtt.client as mqtt
from yamspy import MSPy
import serial # Import for catching serial exceptions
import logging # Keep the import
from drone_logging import setup_logger # Import our new setup function

# --- Configuration ---
SERIAL_PORT = "/dev/ttyACM0"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
LOOP_FREQUENCY = 100
LOOP_TIME = 1.0 / LOOP_FREQUENCY
LOG_FILE = "fc_interface.log" # Log file name

# --- MQTT Topics ---
SENSOR_TOPIC = "drone/sensors"
COMMAND_TOPIC = "drone/commands"
STATUS_TOPIC = "drone/status"
SYSTEM_TOPIC = "drone/system_command" # Listen for system commands

# --- Setup Logger ---
# Use the new centralized logger setup
logger = setup_logger("FC_Interface", LOG_FILE)


# --- Main Logic ---
def on_connect(client, userdata, flags, reason_code, properties):
    """V2 API Callback for when the client connects to the broker."""
    if reason_code == 0:
        logger.info("FC Interface connected to MQTT Broker!")
        # Subscribe to topics
        client.subscribe(COMMAND_TOPIC)
        client.subscribe(SYSTEM_TOPIC) # Subscribe to the new topic
    else:
        logger.error(f"FC Interface failed to connect, return code {reason_code}")

def on_message(client, userdata, msg):
    """Callback for when a message is received on ANY subscribed topic."""
    board = userdata.get('board')
    if not board:
        logger.error("Board not found in userdata!")
        return
        
    payload_str = msg.payload.decode()
    logger.info(f"Received message on topic '{msg.topic}': {payload_str}")
        
    try:
        command_payload = json.loads(payload_str)

        # --- Handle RC Commands ---
        if msg.topic == COMMAND_TOPIC:
            cmds_order = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
            defaults = {'roll': 1500, 'pitch': 1500, 'throttle': 900, 'yaw': 1500, 'aux1': 1000, 'aux2': 1000}
            raw_rc_channels = [command_payload.get(key, defaults[key]) for key in cmds_order]

            if board.send_RAW_RC(raw_rc_channels):
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)
        
        # --- Handle System Commands ---
        elif msg.topic == SYSTEM_TOPIC:
            command = command_payload.get("command")
            if command == "calibrate":
                logger.info("Received CALIBRATE command. Calibrating accelerometer...")
                if board.send_RAW_msg(MSPy.MSPCodes['MSP_ACC_CALIBRATION'], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
                    logger.info("Calibration complete.")
                else:
                    logger.warning("Failed to send calibration command.")

    except (json.JSONDecodeError, KeyError) as e:
        logger.error(f"Could not decode or process command: {payload_str}. Error: {e}")
    except Exception as e:
        logger.error(f"Error in on_message: {e}")


def main():
    """Main function to connect to FC and loop MQTT client."""
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="fc_interface")
    client.on_connect = on_connect
    client.on_message = on_message # A single callback will handle topic logic

    logger.info("Connecting to the Flight Controller...")
    try:
        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board == 1:
                logger.error("Could not connect to the flight controller. Aborting.")
                return

            logger.info("Connected to FC successfully!")
            client.user_data_set({'board': board})
            
            try:
                client.connect(MQTT_BROKER, MQTT_PORT, 60)
            except ConnectionRefusedError:
                logger.error("FC Interface: Connection to MQTT broker refused. Is it running?")
                return

            client.loop_start() 
            
            while True:
                try:
                    start_time = time.time()
                    
                    # --- Sensor Publishing Loop ---
                    if board.send_RAW_msg(MSPy.MSPCodes['MSP_ATTITUDE'], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                    
                    if board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                    sensor_data = {
                        "kinematics": board.SENSOR_DATA.get('kinematics', [0,0,0]),
                        "altitude": board.SENSOR_DATA.get('altitude', 0),
                        "timestamp": time.time()
                    }
                    client.publish(SENSOR_TOPIC, json.dumps(sensor_data))
                    logger.debug(f"Published sensors: {sensor_data}") # Log sensor data

                    ARMED = board.bit_check(board.CONFIG.get('mode', 0), 0)
                    client.publish(STATUS_TOPIC, json.dumps({"armed": ARMED}))
                    logger.debug(f"Published status: armed={ARMED}") # Log status

                    elapsed = time.time() - start_time
                    if elapsed < LOOP_TIME:
                        time.sleep(LOOP_TIME - elapsed)

                except KeyboardInterrupt:
                    logger.info("FC Interface stopping...")
                    break
                except Exception as e:
                    logger.error(f"An error occurred in the FC main loop: {e}")
                    time.sleep(1)
                    break 

            client.loop_stop()
            
    except serial.serialutil.SerialException as e:
        logger.error(f"Serial port error: {e}. Is the drone plugged in at {SERIAL_PORT}?")
    except Exception as e:
        logger.error(f"Failed to initialize MSPy: {e}")


if __name__ == '__main__':
    # Set logger level to DEBUG if you want to see sensor/status messages
    logger.setLevel(logging.DEBUG) # or logging.DEBUG
    main()

