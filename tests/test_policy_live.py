import json
import time
import paho.mqtt.client as mqtt
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
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
LOG_FILE = os.path.join(SCRIPT_DIR, "test_policy_live.log")

def log(msg):
    timestamp = time.strftime("%H:%M:%S")
    formatted = f"{timestamp} - {msg}"
    print(formatted)
    with open(LOG_FILE, "a") as f:
        f.write(formatted + "\n")
        f.flush()

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload = msg.payload.decode()
        log(f"RECV | {topic} | {payload}")
    except Exception as e:
        log(f"ERROR: {e}")

def main():
    # Clear log at start
    with open(LOG_FILE, "w") as f:
        f.write(f"--- TEST START {time.ctime()} ---\n")

    log(f"Starting Policy Monitor. Logging to: {LOG_FILE}")
    
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        log(f"MQTT Connection Failed: {e}")
        return

    # Subscribe to ALL drone topics
    client.subscribe("drone/#")
    log("Subscribed to drone/#")

    client.loop_start()
    time.sleep(2)
    
    log("Triggering AI and Arming...")
    client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": True}))
    client.publish(STATUS_TOPIC, json.dumps({"armed": True}))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log("Shutting down...")
        client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": False}))
        client.publish(STATUS_TOPIC, json.dumps({"armed": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
