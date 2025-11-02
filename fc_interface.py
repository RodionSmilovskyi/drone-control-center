import time
import json
import paho.mqtt.client as mqtt
from yamspy import MSPy
import serial # Import for catching serial exceptions
import logging # Keep the import
from drone_logging import setup_logger # Import our new setup function
from itertools import cycle # To cycle through sensor requests

# --- Configuration ---
SERIAL_PORT = "/dev/ttyACM0"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
CTRL_LOOP_TIME = 1.0 / 100  # 100Hz loop for RC commands (the failsafe heartbeat)
SLOW_LOOP_TIME = 1.0 / 10  # 10Hz loop for polling sensors
LOG_FILE = "fc_interface.log" # Log file name

# --- MQTT Topics ---
SENSOR_TOPIC = "drone/sensors"
COMMAND_TOPIC = "drone/commands"
STATUS_TOPIC = "drone/status"
SYSTEM_TOPIC = "drone/system_command" # Listen for system commands

# --- Setup Logger ---
logger = setup_logger("FC_Interface", LOG_FILE)

# --- Global RC Command State ---
# This dictionary holds the most recent commands.
# It's updated by MQTT and sent to the FC in the main 100Hz loop.
rc_commands = {'roll': 1500, 'pitch': 1500, 'throttle': 900, 'yaw': 1500, 'aux1': 1000, 'aux2': 1000}
CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']


# --- Main Logic ---
def on_connect(client, userdata, flags, reason_code, properties):
    """V2 API Callback for when the client connects to the broker."""
    if reason_code == 0:
        logger.info("FC Interface connected to MQTT Broker!")
        client.subscribe(COMMAND_TOPIC)
        client.subscribe(SYSTEM_TOPIC)
    else:
        logger.error(f"FC Interface failed to connect, return code {reason_code}")

def on_message(client, userdata, msg):
    """Callback for when a message is received on ANY subscribed topic."""
    global rc_commands # We will update the global command state
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
            # Just update the dictionary. DO NOT send the command here.
            rc_commands.update(command_payload)
        
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

            logger.info("Connected to FC successfully! Starting initialization sequence...")
            client.user_data_set({'board': board})

            # --- ADDED: Initialization Sequence (from simpleUI.py) ---
            # This is critical to prevent failsafe on some boards.
            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES', 'MSP_ATTITUDE', 'MSP_ALTITUDE']
            
            for msg in command_list: 
                logger.debug(f"Sending init command: {msg}")
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
            logger.info("Initialization sequence complete.")
            # --- END of Initialization Sequence ---

            try:
                client.connect(MQTT_BROKER, MQTT_PORT, 60)
            except ConnectionRefusedError:
                logger.error("FC Interface: Connection to MQTT broker refused. Is it running?")
                return

            client.loop_start() 
            
            last_ctrl_loop_time = time.time()
            last_slow_loop_time = time.time()
            
            # Cycle through sensor requests
            slow_msgs = cycle(['MSP_ATTITUDE', 'MSP_ALTITUDE', 'MSP_STATUS_EX'])
            
            while True:
                try:
                    current_time = time.time()

                    # --- 100Hz FAILSAFE HEARTBEAT LOOP (from simpleUI.py) ---
                    if (current_time - last_ctrl_loop_time) >= CTRL_LOOP_TIME:
                        # Send the most recent RC commands
                        raw_rc_channels = [rc_commands.get(key, 1500) for key in CMDS_ORDER]
                        if board.send_RAW_RC(raw_rc_channels):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
                            logger.debug("Sent 100Hz RC Heartbeat")
                        
                        last_ctrl_loop_time = current_time

                    # --- 10Hz SENSOR POLLING LOOP (from simpleUI.py) ---
                    if (current_time - last_slow_loop_time) >= SLOW_LOOP_TIME:
                        next_msg = next(slow_msgs)
                        logger.debug(f"Polling sensor: {next_msg}")
                        
                        if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
                        
                        # Process sensor data
                        if next_msg == 'MSP_ATTITUDE' or next_msg == 'MSP_ALTITUDE':
                            sensor_data = {
                                "kinematics": board.SENSOR_DATA.get('kinematics', [0,0,0]),
                                "altitude": board.SENSOR_DATA.get('altitude', 0),
                                "timestamp": time.time()
                            }
                            client.publish(SENSOR_TOPIC, json.dumps(sensor_data))
                            logger.debug(f"Published sensors: {sensor_data}")
                        
                        elif next_msg == 'MSP_STATUS_EX':
                            ARMED = board.bit_check(board.CONFIG.get('mode', 0), 0)
                            client.publish(STATUS_TOPIC, json.dumps({"armed": ARMED}))
                            logger.debug(f"Published status: armed={ARMED}")
                            
                        last_slow_loop_time = current_time

                    # Sleep to prevent busy-waiting
                    time.sleep(0.001)

                except KeyboardInterrupt:
                    logger.info("FC Interface stopping...")
                    break
                except Exception as e:
                    logger.error(f"An error occurred in the FC main loop: {e}")
                    time.sleep(1)
                    # We will try to continue unless it's a serial error
                    if "serial" in str(e).lower():
                        logger.error("Serial error. Exiting.")
                        break

            client.loop_stop()
            
    except serial.serialutil.SerialException as e:
        logger.error(f"Serial port error: {e}. Is the drone plugged in at {SERIAL_PORT}?")
    except Exception as e:
        logger.error(f"Failed to initialize MSPy: {e}")


if __name__ == '__main__':
    logger.setLevel(logging.DEBUG) # or logging.DEBUG
    main()

