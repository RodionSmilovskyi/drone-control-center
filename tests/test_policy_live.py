import json
import time
import paho.mqtt.client as mqtt
import os
import sys

# --- Configuration ---
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883

# --- Log Setup ---
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
LOG_FILE = os.path.join(SCRIPT_DIR, "test_policy_live.log")

def log_msg(text):
    formatted = f"{time.strftime('%H:%M:%S')} | {text}"
    with open(LOG_FILE, "a") as f:
        f.write(formatted + "\n")
        f.flush()

state = {"obs": None, "act": None, "cmd": None}

def on_message(client, userdata, msg):
    global state
    topic = msg.topic
    try:
        raw = msg.payload.decode()
        # SUPER VERBOSE: Log every single packet seen on the network
        log_msg(f"NETWORK | {topic} | {raw}")
        
        payload = json.loads(raw)
        
        if topic == "drone/observation":
            state["obs"] = payload.get("observation")
        elif topic == "drone/target_setpoints":
            state["act"] = payload
        elif topic == "drone/commands":
            state["cmd"] = payload
            
    except Exception as e:
        log_msg(f"DECODE ERROR on {topic}: {e}")

def main():
    # Kill old log
    with open(LOG_FILE, "w") as f:
        f.write(f"--- SUPER VERBOSE LOG START: {time.ctime()} ---\n")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        # Subscribe to EVERYTHING to see where the data is leaking
        client.subscribe("#") 
        client.loop_start()
    except Exception as e:
        print(f"MQTT FAIL: {e}")
        return

    print(f"Passive Network Monitor Running. Target: {LOG_FILE}")

    try:
        while True:
            # Re-publish triggers just in case
            client.publish("drone/ai_mode", json.dumps({"ai_enabled": True}))
            client.publish("drone/status", json.dumps({"armed": True}))
            
            p = state["obs"][6] if (state["obs"] and len(state["obs"]) > 6) else "?"
            print(f"\rPulse:{p} | OBS:{'OK' if state['obs'] else '-'} | ACT:{'OK' if state['act'] else '-'} | CMD:{'OK' if state['cmd'] else '-'}", end="")
            sys.stdout.flush()
            time.sleep(1)
    except KeyboardInterrupt:
        client.loop_stop()

if __name__ == "__main__":
    main()
