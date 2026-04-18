import json
import time
import paho.mqtt.client as mqtt
import os

# --- Configuration ---
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883

# --- Log Setup ---
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
LOG_FILE = os.path.join(SCRIPT_DIR, "test_policy_live.log")

def log_msg(text):
    formatted = f"{time.strftime('%H:%M:%S')} | {text}"
    # Note: We don't print to console here to keep the heartbeat clean
    with open(LOG_FILE, "a") as f:
        f.write(formatted + "\n")
        f.flush()

state = {"obs": None, "act": None, "cmd": None}

def on_message(client, userdata, msg):
    global state
    topic = msg.topic
    try:
        payload = json.loads(msg.payload.decode())
        if topic == "drone/observation":
            state["obs"] = payload.get("observation")
        elif topic == "drone/target_setpoints":
            state["act"] = [payload.get("target_altitude_norm"), 0, 0, 0]
        elif topic == "drone/commands":
            state["cmd"] = [payload.get("throttle"), payload.get("roll"), payload.get("pitch"), payload.get("yaw")]
            
        if state["obs"] and state["act"] and state["cmd"]:
            log_compact_row()
    except Exception as e:
        pass

def log_compact_row():
    # Write to the file
    log_msg(f"FUSED | OBS:{state['obs']} | ACT:{state['act']} | CMD:{state['cmd']}")

def main():
    with open(LOG_FILE, "w") as f:
        f.write(f"--- COMPACT MONITOR START: {time.ctime()} ---\n")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.subscribe("drone/#")
        client.loop_start()
    except Exception as e:
        print(f"MQTT FAIL: {e}")
        return

    print(f"Starting Compact Monitor. Log: {LOG_FILE}")
    print("Will continuously send signals every 2s...")

    try:
        while True:
            # Re-publish signals to catch any late-starting services
            client.publish("drone/ai_mode", json.dumps({"ai_enabled": True}))
            client.publish("drone/status", json.dumps({"armed": True}))
            
            # Print a compact summary of what we have so far
            obs_pulse = state["obs"][6] if (state["obs"] and len(state["obs"]) > 6) else "Wait"
            print(f"\rPulse:{obs_pulse} | OBS:{'OK' if state['obs'] else 'Wait'} | ACT:{'OK' if state['act'] else 'Wait'} | CMD:{'OK' if state['cmd'] else 'Wait'}", end="")
            
            time.sleep(2)
    except KeyboardInterrupt:
        client.publish("drone/ai_mode", json.dumps({"ai_enabled": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
