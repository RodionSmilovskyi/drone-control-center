import json
import time
import paho.mqtt.client as mqtt
import logging

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TARGET_TOPIC = "drone/target_setpoints"
AI_MODE_TOPIC = "drone/ai_mode"
STATUS_TOPIC = "drone/status"
LOG_FILE = "policy_test.log"

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("PolicyMonitor")

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        log_msg = (
            f"\n[POLICY OUTPUT]\n"
            f"  Alt Target: {payload.get('target_altitude_norm')}\n"
            f"  Roll Target: {payload.get('target_roll_norm')}\n"
            f"  Pitch Target: {payload.get('target_pitch_norm')}\n"
            f"  Yaw Target: {payload.get('target_yaw_norm')}"
        )
        logger.info(log_msg)
    except Exception as e:
        logger.error(f"Error decoding message: {e}")

def main():
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe(TARGET_TOPIC)
    client.loop_start()

    print("--- Policy Live Monitor ---")
    print("Enabling AI Mode and Arming to trigger Strategic Agent...")
    client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": True}))
    client.publish(STATUS_TOPIC, json.dumps({"armed": True}))

    print("Monitoring output. Move sensors to see if anything changes (though R/P/Y should stay 0).")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": False}))
        client.publish(STATUS_TOPIC, json.dumps({"armed": False}))
        client.loop_stop()

if __name__ == "__main__":
    main()
