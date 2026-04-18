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

def log(msg):
    with open(LOG_FILE, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')} - {msg}\n")

# Global variables to track the "pulse"
last_obs = None
last_act = None
last_cmd = None

def on_message(client, userdata, msg):
    global last_obs, last_act, last_cmd
    try:
        payload = json.loads(msg.payload.decode())
        
        if msg.topic == "drone/observation":
            last_obs = payload.get("observation")
        elif msg.topic == "drone/target_setpoints":
            last_act = payload
        elif msg.topic == "drone/commands":
            last_cmd = payload
            
        # Every time we get a command, print a summary to the console
        if last_obs and last_act and last_cmd:
            # The 7th element of OBS is our moving timestamp
            pulse = last_obs[6] if len(last_obs) > 6 else 0
            
            # Clear line and print high-visibility summary
            print(f"\r[PULSE:{pulse:4.1f}] ALT:{last_obs[0]:.2f} -> TGT:{last_act['target_altitude_norm']:.1f} -> THR:{last_cmd['throttle']}", end="")
            
            # Log full details to file
            log(f"FUSION | OBS:{last_obs} | ACT:{last_act} | CMD:{last_cmd}")
            
    except Exception as e:
        pass

def main():
    print(f"!!! POLICY MONITOR STARTING !!!")
    print(f"Logging full history to: {LOG_FILE}")
    
    with open(LOG_FILE, "w") as f:
        f.write("--- NEW TEST SESSION ---\n")

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe("drone/#")
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
