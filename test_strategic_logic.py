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
        print(f"\n[RECEIVED] Target Setpoints on {msg.topic}:")
        print(json.dumps(payload, indent=4))
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
    client.loop_start()

    print("--- Strategic Agent Test Script ---")
    print("1. Enabling AI Mode...")
    client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": True}))
    
    print("2. Arming Drone...")
    client.publish(STATUS_TOPIC, json.dumps({"armed": True}))

    print("3. Sending Mock Sensor Data (Simulating movement)...")
    
    # Simulate 5 seconds of data at 10Hz
    for i in range(20):
        # Simulate altitude: 0.3m (30cm)
        # Simulate flow: moving right (dx=10) and forward (dy=5)
        mock_sensors = {
            "altitude": 0.3,
            "flow": {"x": 10, "y": 5},
            "timestamp": time.time()
        }
        client.publish(SENSOR_TOPIC, json.dumps(mock_sensors))
        time.sleep(0.1)

    print("\nTest finished. Check the output above for target setpoints.")
    print("If you don't see 'RECEIVED' messages, check if strategic_agent.py is running.")
    
    client.loop_stop()

if __name__ == "__main__":
    main()
