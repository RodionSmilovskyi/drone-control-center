import time
import json
import paho.mqtt.client as mqtt
from yamspy import MSPy

# --- Configuration ---
SERIAL_PORT = "/dev/ttyACM0"  # Match your flight controller's serial port
MQTT_BROKER = "localhost"   # The Pi Zero is the broker
MQTT_PORT = 1883

# --- MQTT Topics ---
SENSOR_TOPIC = "drone/sensors"
COMMAND_TOPIC = "drone/commands"
STATUS_TOPIC = "drone/status"

# --- Main Logic ---
def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        print("Connected to MQTT Broker!")
        # Subscribe to the command topic to receive instructions
        client.subscribe(COMMAND_TOPIC)
    else:
        print(f"Failed to connect, return code {rc}\n")

def on_message(client, userdata, msg):
    """Callback for when a message is received on a subscribed topic."""
    board = userdata['board']
    try:
        # Decode the command from the MQTT message
        command_payload = json.loads(msg.payload.decode())
        
        # This order is critical and must match your FC's AETR config
        cmds_order = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
        raw_rc_channels = [command_payload.get(key, 1500) for key in cmds_order]

        # Send the raw RC commands to the flight controller
        if board.send_RAW_RC(raw_rc_channels):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)

    except (json.JSONDecodeError, KeyError) as e:
        print(f"Could not decode or process command: {e}")

def main():
    """Main function to connect to FC and loop MQTT client."""
    client = mqtt.Client(client_id="fc_interface")
    client.on_connect = on_connect
    client.on_message = on_message

    print("Connecting to the Flight Controller...")
    with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
        if board == 1:
            print("Could not connect to the flight controller. Aborting.")
            return

        print("Connected to FC successfully!")
        client.user_data_set({'board': board})
        
        try:
            client.connect(MQTT_BROKER, MQTT_PORT, 60)
        except ConnectionRefusedError:
            print("Connection to MQTT broker refused. Is it running?")
            return

        # Loop to continuously read sensors and publish data
        client.loop_start() # Handles reconnects and processes messages in a background thread
        
        while True:
            try:
                # Request Attitude and Altitude data from the FC
                if board.send_RAW_msg(MSPy.MSPCodes['MSP_ATTITUDE'], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
                
                if board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)

                # Prepare the sensor data payload
                sensor_data = {
                    "kinematics": board.SENSOR_DATA.get('kinematics', [0,0,0]),
                    "altitude": board.SENSOR_DATA.get('altitude', 0),
                    "timestamp": time.time()
                }

                # Publish sensor data to the MQTT topic
                client.publish(SENSOR_TOPIC, json.dumps(sensor_data))

                # Also publish armed status
                ARMED = board.bit_check(board.CONFIG.get('mode', 0), 0)
                client.publish(STATUS_TOPIC, json.dumps({"armed": ARMED}))

                time.sleep(1/100)  # Run at approximately 100Hz

            except Exception as e:
                print(f"An error occurred in the main loop: {e}")
                break

        client.loop_stop()

if __name__ == '__main__':
    main()
