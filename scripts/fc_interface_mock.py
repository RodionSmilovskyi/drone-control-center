import logging
import time
import json
import paho.mqtt.client as mqtt
from drone_logging import setup_logger

# --- Configuration ---
MQTT_BROKER = "localhost"   # Connect to the broker on our local machine
MQTT_PORT = 1883
LOOP_FREQUENCY = 100  # Run the simulation at 100Hz
LOOP_TIME = 1.0 / LOOP_FREQUENCY
LOG_FILE = "mock_fc.log" # Log file name

# --- MQTT Topics ---
SENSOR_TOPIC = "drone/sensors"
COMMAND_TOPIC = "drone/commands"
STATUS_TOPIC = "drone/status"
SYSTEM_TOPIC = "drone/system_command"

logger = setup_logger("Mock_FC", LOG_FILE)

# --- Simulation State ---
# This dictionary will hold the drone's simulated state
sim_state = {
    "kinematics": [0, 0, 0],  # [roll, pitch, yaw]
    "altitude": 0.0,
    "armed": True,
    "last_command": {
        "throttle": 900, "roll": 1500, "pitch": 1500, "yaw": 1500,
        "aux1": 1000, "aux2": 1000
    }
}

def on_connect(client, userdata, flags, rc, properties):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        print("Mock FC connected to MQTT Broker!")
        logger.info("Mock FC connected to MQTT Broker!")
        # Subscribe to the command topic to receive instructions
        client.subscribe(COMMAND_TOPIC)
    else:
        print(f"Failed to connect, return code {rc}\n")
        logger.error(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    """Callback for when a command is received."""
    global sim_state
    payload_str = msg.payload.decode()
    logger.info(f"Received message on topic '{msg.topic}': {payload_str}")
    
    try:
        command_payload = json.loads(msg.payload.decode())

        # --- Handle RC Commands ---
        if msg.topic == COMMAND_TOPIC:
            sim_state["last_command"].update(command_payload)
            if sim_state["last_command"].get("aux1", 1000) > 1500:
                sim_state["armed"] = True
            else:
                sim_state["armed"] = False
                sim_state["altitude"] = 0.0
        
        # --- Handle System Commands ---
        elif msg.topic == SYSTEM_TOPIC:
            command = command_payload.get("command")
            if command == "calibrate":
                print("Mock FC received CALIBRATE command. Simulating calibration.")
                logger.info("Received CALIBRATE command. Simulating calibration.")
                # Reset kinematics to 0
                sim_state["kinematics"] = [0, 0, 0]

    except (json.JSONDecodeError, KeyError) as e:
        print(f"Could not decode or process command: {e}")
        logger.error(f"Could not decode or process command: {payload_str}. Error: {e}")

def update_simulation():
    """Update the drone's physics based on the last command."""
    global sim_state
    
    if not sim_state["armed"]:
        return # Do nothing if not armed

    # --- Simple Physics Simulation ---
    # 1. Update Altitude
    # Map 900-2000 throttle to a vertical velocity
    throttle = sim_state["last_command"].get("throttle", 900)
    # 1500 is hover, > 1500 is up, < 1500 is down
    vertical_velocity = (throttle - 1500) / 500.0  # Gives a range of -1.2 to 1.0 m/s
    sim_state["altitude"] += vertical_velocity * LOOP_TIME
    
    # Don't go below ground
    if sim_state["altitude"] < 0:
        sim_state["altitude"] = 0

    # 2. Update Kinematics (Roll/Pitch)
    # Map 1000-2000 roll/pitch to an angle
    roll_cmd = sim_state["last_command"].get("roll", 1500)
    pitch_cmd = sim_state["last_command"].get("pitch", 1500)
    sim_state["kinematics"][0] = (roll_cmd - 1500) / 500.0 * 45 # Max 45 degrees
    sim_state["kinematics"][1] = (pitch_cmd - 1500) / 500.0 * 45 # Max 45 degrees


def main():
    """Main function to setup MQTT client and run the mock loop."""
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="mock_fc")
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except ConnectionRefusedError:
        print("Connection to MQTT broker refused. Is it running? (Try 'sudo service mosquitto start')")
        logger.error("Connection to MQTT broker refused. Is it running? (Try 'sudo service mosquitto start')")
        return

    client.loop_start() # Handles network traffic in a background thread

    print("Mock FC running... Publishing to topics. Press Ctrl+C to stop.")
    logger.info("Mock FC running... Publishing to topics. Press Ctrl+C to stop.")
    while True:
        try:
            start_time = time.time()
            
            # 1. Update the simulation physics
            update_simulation()
            
            # 2. Prepare the sensor data payload
            sensor_data = {
                "kinematics": sim_state["kinematics"],
                "altitude": sim_state["altitude"],
                "timestamp": time.time()
            }
            status_data = {"armed": sim_state["armed"]}

            # 3. Publish sensor and status data
            client.publish(SENSOR_TOPIC, json.dumps(sensor_data))
            client.publish(STATUS_TOPIC, json.dumps(status_data))
            
            logger.debug(f"Published sensors: {sensor_data}")
            logger.debug(f"Published status: {status_data}")
            
            # 4. Sleep to maintain the loop frequency
            elapsed = time.time() - start_time
            if elapsed < LOOP_TIME:
                time.sleep(LOOP_TIME - elapsed)
                
        except KeyboardInterrupt:
            print("\nShutting down Mock FC...")
            logger.info("\nShutting down Mock FC...")
            break

    client.loop_stop()

if __name__ == '__main__':
    logger.setLevel(logging.DEBUG)
    main()
