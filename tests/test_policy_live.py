import json
import time
import paho.mqtt.client as mqtt
import logging
import os

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TARGET_TOPIC = "drone/target_setpoints"
AI_MODE_TOPIC = "drone/ai_mode"
STATUS_TOPIC = "drone/status"
OBSERVATION_TOPIC = "drone/observation"
COMMAND_TOPIC = "drone/commands"

# --- Log Setup ---
# Dedicated log file in the tests folder
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_FILE = os.path.join(BASE_DIR, "test_policy_live.log")

# Setup a single logger
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("PolicyMonitor")

# Global state to keep track of latest data for fusion
state = {
    "observation": None,
    "action": None,
    "commands": None
}

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        
        if msg.topic == OBSERVATION_TOPIC:
            state["observation"] = payload.get("observation")
            
        elif msg.topic == TARGET_TOPIC:
            state["action"] = payload
            
        elif msg.topic == COMMAND_TOPIC:
            state["commands"] = payload
            # Log the fused state whenever a new command (the final step) is received
            log_fused_state()
            
    except Exception as e:
        logger.error(f"Error decoding message on {msg.topic}: {e}")

def log_fused_state():
    """Logs OBS, ACT, and RC together in one dedicated log entry."""
    if state["observation"] and state["action"] and state["commands"]:
        log_entry = (
            f"\n[POLICY FUSION]\n"
            f"  OBS: {state['observation']}\n"
            f"  ACT: Alt:{state['action'].get('target_altitude_norm')}, R:{state['action'].get('target_roll_norm')}, P:{state['action'].get('target_pitch_norm')}, Y:{state['action'].get('target_yaw_norm')}\n"
            f"  RC : T:{state['commands'].get('throttle')}, R:{state['commands'].get('roll')}, P:{state['commands'].get('pitch')}, Y:{state['commands'].get('yaw')}"
        )
        logger.info(log_entry)

def main():
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe(TARGET_TOPIC)
    client.subscribe(OBSERVATION_TOPIC)
    client.subscribe(COMMAND_TOPIC)
    client.loop_start()

    print(f"--- Policy Live Monitor ---")
    print(f"Logging to: {LOG_FILE}")
    print("Enabling AI Mode and Arming to trigger Strategic Agent...")
    client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": True}))
    client.publish(STATUS_TOPIC, json.dumps({"armed": True}))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": False}))
        client.publish(STATUS_TOPIC, json.dumps({"armed": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
