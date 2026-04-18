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
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_policy_live.log")

logger = logging.getLogger("PolicyMonitor")
logger.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(message)s')

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
        raw_payload = msg.payload.decode()
        logger.info(f"RAW RECV | Topic: {msg.topic} | Data: {raw_payload}")
        
        payload = json.loads(raw_payload)
        
        if msg.topic == OBSERVATION_TOPIC:
            state["observation"] = payload.get("observation")
        elif msg.topic == TARGET_TOPIC:
            state["action"] = payload
        elif msg.topic == COMMAND_TOPIC:
            state["commands"] = payload
            
        if state["observation"] and state["action"] and state["commands"]:
            log_fused_state()
            
        # Immediate flush
        for handler in logger.handlers:
            handler.flush()
            
    except Exception as e:
        logger.error(f"Error on {msg.topic}: {e}")

def log_fused_state():
    log_entry = (
        f"\n[POLICY FUSION SUCCESS]\n"
        f"  OBS: {state['observation']}\n"
        f"  ACT: {state['action']}\n"
        f"  RC : {state['commands']}\n"
    )
    logger.info(log_entry)

def main():
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    
    # Subscribe to everything relevant
    topics = [TARGET_TOPIC, OBSERVATION_TOPIC, COMMAND_TOPIC, STATUS_TOPIC, AI_MODE_TOPIC]
    for t in topics:
        client.subscribe(t)
        logger.info(f"Subscribed to {t}")

    client.loop_start()
    time.sleep(1)
    
    logger.info("Triggering AI and Arming...")
    client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": True}))
    client.publish(STATUS_TOPIC, json.dumps({"armed": True}))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": False}))
        client.publish(STATUS_TOPIC, json.dumps({"armed": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
