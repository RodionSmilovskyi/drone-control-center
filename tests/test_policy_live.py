import json
import time
import paho.mqtt.client as mqtt
import os

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# --- Log Setup ---
# Use an absolute path that works on the Pi
LOG_FILE = "/home/rodion/drone/tests/test_policy_live.log"

def log_to_file(msg):
    with open(LOG_FILE, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')} | {msg}\n")
        f.flush()

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        data = msg.payload.decode()
        # Print to console for immediate feedback
        print(f"RECV: {topic}")
        # Log to file for deep analysis
        log_to_file(f"{topic} | {data}")
    except Exception as e:
        print(f"Error: {e}")

def main():
    print(f"--- STARTING RAW POLICY MONITOR ---")
    print(f"LOGGING EVERY MESSAGE TO: {LOG_FILE}")
    
    # Initialize log
    with open(LOG_FILE, "w") as f:
        f.write(f"--- SESSION START: {time.ctime()} ---\n")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"MQTT FAILED: {e}")
        return

    client.subscribe("drone/#")
    client.loop_start()

    # Trigger the system
    time.sleep(2)
    print("SENDING AI_ENABLE AND ARMED...")
    client.publish("drone/ai_mode", json.dumps({"ai_enabled": True}))
    client.publish("drone/status", json.dumps({"armed": True}))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        client.publish("drone/ai_mode", json.dumps({"ai_enabled": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
