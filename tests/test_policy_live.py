import json
import time
import paho.mqtt.client as mqtt
import logging
import os
import sys

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TARGET_TOPIC = "drone/target_setpoints"
AI_MODE_TOPIC = "drone/ai_mode"
STATUS_TOPIC = "drone/status"
OBSERVATION_TOPIC = "drone/observation"
COMMAND_TOPIC = "drone/commands"

# --- Log Setup ---
# Use absolute path for safety
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_policy_live.log")

logger = logging.getLogger("PolicyMonitor")
logger.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(message)s')

# Clear old log and setup handlers
fh = logging.FileHandler(LOG_FILE, mode='w')
fh.setFormatter(formatter)
logger.addHandler(fh)

ch = logging.StreamHandler(sys.stdout)
ch.setFormatter(formatter)
logger.addHandler(ch)

# Global state
state = {
    "observation": None,
    "action": None,
    "commands": None
}

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        topic = msg.topic
        
        if topic == OBSERVATION_TOPIC:
            state["observation"] = payload.get("observation")
            logger.info(f"DEBUG: Received OBS from {topic}")
        elif topic == TARGET_TOPIC:
            state["action"] = payload
            logger.info(f"DEBUG: Received ACT from {topic}")
        elif topic == COMMAND_TOPIC:
            state["commands"] = payload
            logger.info(f"DEBUG: Received CMD from {topic}")
            
        # Try to log fused state every time we get a command
        if state["observation"] and state["action"] and state["commands"]:
            log_fused_state()
        else:
            missing = [k for k, v in state.items() if v is None]
            logger.info(f"DEBUG: Waiting for: {missing}")

        # Force flush to disk
        for handler in logger.handlers:
            handler.flush()
            
    except Exception as e:
        logger.error(f"Error decoding message on {msg.topic}: {e}")

def log_fused_state():
    log_entry = (
        f"\n[POLICY FUSION]\n"
        f"  OBS: {state['observation']}\n"
        f"  ACT: Alt:{state['action'].get('target_altitude_norm')}, R:{state['action'].get('target_roll_norm')}, P:{state['action'].get('target_pitch_norm')}, Y:{state['action'].get('target_yaw_norm')}\n"
        f"  RC : T:{state['commands'].get('throttle')}, R:{state['commands'].get('roll')}, P:{state['commands'].get('pitch')}, Y:{state['commands'].get('yaw')}\n"
    )
    logger.info(log_entry)

def main():
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        logger.error(f"Failed to connect to MQTT: {e}")
        return

    client.subscribe([(TARGET_TOPIC, 0), (OBSERVATION_TOPIC, 0), (COMMAND_TOPIC, 0)])
    client.loop_start()

    logger.info(f"--- Policy Live Monitor Started ---")
    logger.info(f"Logging to: {LOG_FILE}")
    
    time.sleep(1)
    
    logger.info("Sending AI_MODE and ARMED signals...")
    client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": True}))
    client.publish(STATUS_TOPIC, json.dumps({"armed": True}))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": False}))
        client.publish(STATUS_TOPIC, json.dumps({"armed": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
