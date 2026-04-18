import json
import time
import paho.mqtt.client as mqtt
import os
import sys

# --- Configuration ---
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
LOG_FILE = "/home/rodion/drone/tests/test_policy_live.log"

# Internal State to hold the latest fragments
state = {
    "obs": None,
    "act": None,
    "cmd": None
}

def log_msg(text):
    formatted = f"{time.strftime('%H:%M:%S')} | {text}"
    print(formatted)
    with open(LOG_FILE, "a") as f:
        f.write(formatted + "\n")
        f.flush()

def on_connect(client, userdata, flags, rc):
    client.subscribe("drone/#")

def on_message(client, userdata, msg):
    global state
    topic = msg.topic
    try:
        payload = json.loads(msg.payload.decode())
        
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
            # Use Command arrival as the trigger for a fused row
            log_compact_row()
            
    except Exception as e:
        pass

def log_compact_row():
    # Only log if we have at least one valid sample for each
    if state["obs"] is not None and state["act"] is not None and state["cmd"] is not None:
        # Elements are: [OBS] | [ACT] | [RC]
        obs_str = f"OBS:{state['obs']}"
        act_str = f"ACT:{state['act']}"
        cmd_str = f"CMD:{state['cmd']}"
        log_msg(f"FUSED | {obs_str} | {act_str} | {cmd_str}")

def main():
    with open(LOG_FILE, "w") as f:
        f.write(f"--- COMPACT FUSED LOG START: {time.ctime()} ---\n")

    # Use v1 API for maximum compatibility on Pi
    client = mqtt.Client(client_id="compact_monitor", clean_session=True)
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"MQTT FAIL: {e}")
        return

    print(f"Compact Monitor Running. Log: {LOG_FILE}")

    try:
        while True:
            # Continuously pulse triggers
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
