import json
import time
import paho.mqtt.client as mqtt
import os
import sys

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# --- Log Setup ---
# Force a clean, absolute path on the Pi
LOG_FILE = "/home/rodion/drone/tests/test_policy_live.log"

def log_msg(text):
    # Print to console for immediate feedback in tmux
    print(text)
    # Append to file for fine-tuning
    with open(LOG_FILE, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')} | {text}\n")
        f.flush()

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload = msg.payload.decode()
        # Log EVERYTHING that happens on the drone topics
        log_msg(f"RECV | {topic} | {payload}")
    except Exception as e:
        print(f"Error: {e}")

def main():
    # Clear log file at start
    with open(LOG_FILE, "w") as f:
        f.write(f"--- SESSION START: {time.ctime()} ---\n")
    
    log_msg(f"MONITOR STARTING. LOGGING TO: {LOG_FILE}")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        log_msg(f"MQTT CONNECTION FAILED: {e}")
        return

    # Subscribe to ALL drone topics to see the full data flow
    client.subscribe("drone/#")
    client.loop_start()

    # Wait for everything else to stabilize
    time.sleep(2)
    log_msg("ENABLING AI MODE AND ARMING...")
    client.publish("drone/ai_mode", json.dumps({"ai_enabled": True}))
    client.publish("drone/status", json.dumps({"armed": True}))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log_msg("SHUTTING DOWN...")
        client.publish("drone/ai_mode", json.dumps({"ai_enabled": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
