import json
import time
import paho.mqtt.client as mqtt

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
SENSOR_TOPIC = "drone/sensors"
STATUS_TOPIC = "drone/status"
AI_MODE_TOPIC = "drone/ai_mode"
TARGET_TOPIC = "drone/target_setpoints"

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        if msg.topic == TARGET_TOPIC:
            print(f"\n[STRATEGIC OUTPUT] Target Setpoints:")
            print(json.dumps(payload, indent=4))
        elif msg.topic == SENSOR_TOPIC:
            # Optional: print incoming real sensor data to confirm it's being received
            # print(f"Sensor Data: {payload}")
            pass
    except Exception as e:
        print(f"Error decoding message: {e}")

def main():
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"Failed to connect to MQTT: {e}")
        return

    client.subscribe(TARGET_TOPIC)
    client.subscribe(SENSOR_TOPIC) # Subscribe to see if sensors are active
    client.loop_start()

    print("--- Strategic Agent LIVE TEST Script ---")
    print("This script enables AI mode and arming to let Strategic Agent process REAL sensor data.")
    print("Make sure 'python3 sensors.py' and 'python3 strategic_agent.py' are running.")
    
    print("\n1. Enabling AI Mode...")
    client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": True}))
    
    print("2. Arming Drone (Simulated for Agent logic)...")
    client.publish(STATUS_TOPIC, json.dumps({"armed": True}))

    print("\nListening for 30 seconds. Move the drone/sensor to see changes in targets.")
    try:
        time.sleep(30)
    except KeyboardInterrupt:
        pass

    print("\n3. Disabling AI Mode and Disarming...")
    client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": False}))
    client.publish(STATUS_TOPIC, json.dumps({"armed": False}))
    
    client.loop_stop()

if __name__ == "__main__":
    main()
