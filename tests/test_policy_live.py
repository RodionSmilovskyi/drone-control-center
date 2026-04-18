import json
import time
import paho.mqtt.client as mqtt
import os

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# --- Log Setup ---
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
LOG_FILE = os.path.join(SCRIPT_DIR, "test_policy_live.log")

def log_msg(text):
    print(text)
    with open(LOG_FILE, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')} | {text}\n")
        f.flush()

# Internal State to hold the latest fragments
state = {
    "obs": "None",
    "act": "None",
    "cmd": "None"
}

def on_message(client, userdata, msg):
    global state
    try:
        topic = msg.topic
        payload = json.loads(msg.payload.decode())
        
        if topic == "drone/observation":
            state["obs"] = payload.get("observation", [])
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
            # Use the arrival of a Command (the final step) as the trigger to print the row
            log_compact_row()
            
    except Exception as e:
        pass

def log_compact_row():
    # Format: [OBS_ALT, OBS_SHIFT_X, OBS_SHIFT_Y, OBS_VEL_X, OBS_VEL_Y, GOAL, PULSE] | [TGT_ALT, TGT_R, TGT_P, TGT_Y] | [RC_T, RC_R, RC_P, RC_Y]
    obs_str = str(state["obs"])
    act_str = str(state["act"])
    cmd_str = str(state["cmd"])
    
    log_msg(f"FUSED | OBS: {obs_str} | ACT: {act_str} | CMD: {cmd_str}")

def main():
    with open(LOG_FILE, "w") as f:
        f.write(f"--- COMPACT LOG START: {time.ctime()} ---\n")
    
    print(f"Starting Compact Monitor. Logging to: {LOG_FILE}")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe([("drone/observation", 0), ("drone/target_setpoints", 0), ("drone/commands", 0)])
    client.loop_start()

    time.sleep(1)
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
