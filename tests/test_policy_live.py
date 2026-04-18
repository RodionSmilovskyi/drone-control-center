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
    # Keep console clean, only write to file
    with open(LOG_FILE, "a") as f:
        f.write(formatted + "\n")
        f.flush()

state = {"obs": None, "act": None, "cmd": None}

def on_message(client, userdata, msg):
    global state
    topic = msg.topic
    try:
        raw = msg.payload.decode()
        # TRY to parse, but don't crash if it's weird
        try:
            payload = json.loads(raw)
        except Exception:
            payload = {"raw": raw} # Fallback
        
        if topic == "drone/observation":
            state["obs"] = payload.get("observation")
        elif topic == "drone/target_setpoints":
            # Just store the raw dict for flexibility
            state["act"] = payload
        elif topic == "drone/commands":
            # Just store the raw dict
            state["cmd"] = payload
            
        # Log if we have all pieces
        if state["obs"] and state["act"] and state["cmd"]:
            log_compact_row()
            
    except Exception as e:
        print(f"\nMONITOR ERROR on {topic}: {e}")

def log_compact_row():
    # Use .get to be safe against different payload formats
    act = state["act"]
    cmd = state["cmd"]
    obs = state["obs"]
    
    # Compact string for the log file
    row = f"FUSED | OBS:{obs} | ACT:[{act.get('target_altitude_norm')},{act.get('target_roll_norm')},{act.get('target_pitch_norm')}] | RC:[{cmd.get('throttle')},{cmd.get('roll')},{cmd.get('pitch')}]"
    log_msg(row)

def main():
    with open(LOG_FILE, "w") as f:
        f.write(f"--- SESSION START: {time.ctime()} ---\n")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        # Subscribe individually to ensure registration
        client.subscribe("drone/observation")
        client.subscribe("drone/target_setpoints")
        client.subscribe("drone/commands")
        client.loop_start()
    except Exception as e:
        print(f"MQTT FAIL: {e}")
        return

    print(f"Compact Monitor Running. Target: {LOG_FILE}")

    try:
        while True:
            # Continuously pulse the triggers
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
