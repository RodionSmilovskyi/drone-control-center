import json
import time
import paho.mqtt.client as mqtt
import os

# --- Configuration ---
MQTT_BROKER = "127.0.0.1" # Explicit IPv4
MQTT_PORT = 1883

# --- Log Setup ---
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
LOG_FILE = os.path.join(SCRIPT_DIR, "test_policy_live.log")

def log_msg(text):
    formatted = f"{time.strftime('%H:%M:%S')} | {text}"
    print(formatted)
    with open(LOG_FILE, "a") as f:
        f.write(formatted + "\n")
        f.flush()

state = {"obs": None, "act": None, "cmd": None}

def on_message(client, userdata, msg):
    global state
    topic = msg.topic
    try:
        # VISUAL HEARTBEAT: Show every message that hits the monitor
        print(f"\rRECV: {topic}          ", end="")
        
        payload = json.loads(msg.payload.decode())
        if topic == "drone/observation":
            state["obs"] = payload.get("observation")
        elif topic == "drone/target_setpoints":
            state["act"] = [payload.get("target_altitude_norm"), 0, 0, 0]
        elif topic == "drone/commands":
            state["cmd"] = [payload.get("throttle"), 1500, 1500, 1500]
            
        if state["obs"] and state["act"] and state["cmd"]:
            log_compact_row()
    except Exception as e:
        print(f"Error: {e}")

def log_compact_row():
    log_msg(f"FUSED | OBS:{state['obs']} | ACT:{state['act']} | CMD:{state['cmd']}")

def main():
    with open(LOG_FILE, "w") as f:
        f.write(f"--- DEBUG START: {time.ctime()} ---\n")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.subscribe("drone/#") # Subscribe to EVERYTHING
        client.loop_start()
    except Exception as e:
        print(f"MQTT FAIL: {e}")
        return

    time.sleep(1)
    client.publish("drone/ai_mode", json.dumps({"ai_enabled": True}))
    client.publish("drone/status", json.dumps({"armed": True}))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        client.loop_stop()

if __name__ == "__main__":
    main()
