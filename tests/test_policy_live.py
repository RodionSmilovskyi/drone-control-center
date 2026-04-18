import json
import time
import paho.mqtt.client as mqtt
import os
import sys

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# --- Log Setup ---
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
LOG_FILE = os.path.join(SCRIPT_DIR, "test_policy_live.log")

def log_msg(text):
    # Print with timestamp to console
    formatted = f"{time.strftime('%H:%M:%S')} | {text}"
    print(formatted)
    # Write to file
    try:
        with open(LOG_FILE, "a") as f:
            f.write(formatted + "\n")
            f.flush()
    except Exception as e:
        print(f"FAILED TO WRITE TO LOG: {e}")

# Internal State to hold the latest fragments
state = {
    "obs": None,
    "act": None,
    "cmd": None
}

def on_message(client, userdata, msg):
    global state
    topic = msg.topic
    try:
        payload_str = msg.payload.decode()
        payload = json.loads(payload_str)
        
        if topic == "drone/observation":
            state["obs"] = payload.get("observation")
        elif topic == "drone/target_setpoints":
            state["act"] = [
                payload.get("target_altitude_norm"),
                payload.get("target_roll_norm"),
                payload.get("target_pitch_norm"),
                payload.get("target_yaw_norm")
            ]
        elif topic == "drone/commands":
            state["cmd"] = [
                payload.get("throttle"),
                payload.get("roll"),
                payload.get("pitch"),
                payload.get("yaw")
            ]
            # Trigger row logging
            log_compact_row()
            
    except Exception as e:
        print(f"ERROR on {topic}: {e}")

def log_compact_row():
    # Only log if we have at least one valid sample for each
    if state["obs"] is not None and state["act"] is not None and state["cmd"] is not None:
        obs_str = str(state["obs"])
        act_str = str(state["act"])
        cmd_str = str(state["cmd"])
        log_msg(f"FUSED | OBS: {obs_str} | ACT: {act_str} | CMD: {cmd_str}")

def main():
    # Force clean start of log
    try:
        with open(LOG_FILE, "w") as f:
            f.write(f"--- COMPACT LOG START: {time.ctime()} ---\n")
    except Exception as e:
        print(f"CRITICAL: Could not initialize log file at {LOG_FILE}: {e}")

    print(f"Starting Compact Monitor. Target Log: {LOG_FILE}")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"MQTT Connection Failed: {e}")
        return

    # Use wildcard to ensure we see EVERYTHING during debug
    client.subscribe("drone/#")
    client.loop_start()

    time.sleep(2)
    print("SENDING AI_ENABLE AND ARMED...")
    client.publish("drone/ai_mode", json.dumps({"ai_enabled": True}))
    client.publish("drone/status", json.dumps({"armed": True}))

    try:
        while True:
            # Print a status heartbeat to console to show script is alive
            missing = [k for k, v in state.items() if v is None]
            if missing:
                print(f"\rWaiting for topics: {missing}", end="")
            time.sleep(1)
    except KeyboardInterrupt:
        client.publish("drone/ai_mode", json.dumps({"ai_enabled": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
