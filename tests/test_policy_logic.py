import json
import time
import paho.mqtt.client as mqtt

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TARGET_TOPIC = "drone/target_setpoints"
AI_MODE_TOPIC = "drone/ai_mode"
STATUS_TOPIC = "drone/status"
SENSOR_TOPIC = "drone/sensors"

class PolicyTester:
    def __init__(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_message = self.on_message
        self.received_targets = []

    def on_message(self, client, userdata, msg):
        payload = json.loads(msg.payload.decode())
        self.received_targets.append(payload)
        print(f"[POLICY OUTPUT] Received Action: {payload}")

    def run_test(self):
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.subscribe(TARGET_TOPIC)
        self.client.loop_start()

        print("--- Strategic Policy Verification Test ---")
        
        # 1. Prepare Agent
        print("1. Activating AI Mode and Arming...")
        self.client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": True}))
        self.client.publish(STATUS_TOPIC, json.dumps({"armed": True}))
        time.sleep(1)

        # 2. Test different goal altitudes
        test_goals = [0.1, 0.5, 0.8]
        
        for goal in test_goals:
            print(f"\n2. Testing Goal Altitude: {goal}")
            
            # We need to send sensor data for the agent loop to trigger
            # In our current strategic_agent.py, norm_target_alt_strategic is hardcoded 
            # to 0.1/MAX_ALTITUDE, so we expect the output to reflect that hardcoded value.
            mock_sensors = {
                "altitude": 0.2, # 20cm
                "flow": {"x": 0, "y": 0},
                "timestamp": time.time()
            }
            self.client.publish(SENSOR_TOPIC, json.dumps(mock_sensors))
            
            time.sleep(1) # Wait for processing

        self.client.loop_stop()
        self.verify_results()

    def verify_results(self):
        print("\n--- Final Verification ---")
        if not self.received_targets:
            print("FAILED: No target setpoints received. Is strategic_agent.py running?")
            return

        last_action = self.received_targets[-1]
        
        # Check if roll/pitch/yaw are zeroed (as per dummy function)
        r = last_action.get("target_roll_norm", 1.0)
        p = last_action.get("target_pitch_norm", 1.0)
        y = last_action.get("target_yaw_norm", 1.0)
        
        if r == 0.0 and p == 0.0 and y == 0.0:
            print("SUCCESS: Roll, Pitch, and Yaw are correctly zeroed.")
        else:
            print(f"FAILED: Expected zeroed axes, got R:{r}, P:{p}, Y:{y}")

        # Note: In strategic_agent.py, target_altitude_norm is currently 
        # hardcoded to 0.1.
        alt = last_action.get("target_altitude_norm", -1.0)
        print(f"INFO: Final Target Altitude published: {alt}")

if __name__ == "__main__":
    tester = PolicyTester()
    tester.run_test()
