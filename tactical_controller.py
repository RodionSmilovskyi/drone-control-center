import time
import json
import paho.mqtt.client as mqtt
import numpy as np
import logging
from drone_logging import setup_logger
from flight_controller import FlightController # Import YOUR FlightController class

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
LOG_FILE = "tactical_controller.log"

# --- Normalization Constants (Must match Strategic Agent) ---
MAX_ALTITUDE = 2.0
MAX_ANGLE = 50.0
MAX_YAW_ANGLE = 180.0

# --- MQTT Topics ---
SENSOR_TOPIC = "drone/sensors"
TARGET_TOPIC = "drone/target_setpoints" # Expects NORMALIZED targets
STATUS_TOPIC = "drone/status"
AI_MODE_TOPIC = "drone/ai_mode" 
COMMAND_TOPIC = "drone/commands" # Publishes REAL RC commands

# --- Setup Logger ---
logger = setup_logger("Tactical_Controller", LOG_FILE)

# --- Controller State ---
class TacticalControllerWrapper:
    def __init__(self):
        # --- Instantiate YOUR FlightController ---
        self.fc = FlightController()
        
        # --- State Variables ---
        self.latest_norm_targets = None
        self.is_armed = False
        self.ai_mode_enabled = False
        self.last_compute_time = time.time()

    def reset(self):
        """Resets the underlying PID controllers."""
        self.fc.reset()
        logger.info("PID controllers reset.")

    def set_normalized_targets(self, norm_targets):
        """Update the setpoints from the strategic agent."""
        self.latest_norm_targets = norm_targets
        logger.debug(f"Normalized setpoints updated: {norm_targets}")

    def normalize_sensor_data(self, sensor_data):
        """Normalizes raw sensor data for the FlightController class."""
        raw_altitude = sensor_data.get("altitude", 0.0)
        raw_kinematics = sensor_data.get("kinematics", [0, 0, 0])
        
        norm_alt = np.clip(raw_altitude / MAX_ALTITUDE, 0.0, 1.0)
        norm_roll = np.clip(raw_kinematics[0] / MAX_ANGLE, -1.0, 1.0)
        norm_pitch = np.clip(raw_kinematics[1] / MAX_ANGLE, -1.0, 1.0)
        norm_yaw = np.clip(raw_kinematics[2] / MAX_YAW_ANGLE, -1.0, 1.0)
        
        return norm_alt, norm_roll, norm_pitch, norm_yaw

    def compute_rc_commands(self, sensor_data):
        """
        Takes raw sensor data, normalizes it, and computes RC commands.
        """
        if not self.is_armed or not self.ai_mode_enabled or self.latest_norm_targets is None:
            if not self.is_armed:
                self.reset()
            return None 

        # --- 1. Normalize Current State ---
        norm_alt, norm_roll, norm_pitch, norm_yaw = self.normalize_sensor_data(sensor_data)
        
        # --- 2. Get Stored Normalized Target ---
        norm_target_alt = self.latest_norm_targets.get("target_altitude_norm", 0.0)
        norm_target_roll = self.latest_norm_targets.get("target_roll_norm", 0.0)
        norm_target_pitch = self.latest_norm_targets.get("target_pitch_norm", 0.0)
        norm_target_yaw = self.latest_norm_targets.get("target_yaw_norm", 0.0)

        # --- 3. Calculate dt ---
        current_time = time.time()
        dt = current_time - self.last_compute_time
        self.last_compute_time = current_time
        if dt <= 0: dt = 1.0 / 100 # Avoid division by zero, default to 100Hz

        # --- 4. Call YOUR FlightController's compute method ---
        # Prepare inputs as numpy arrays
        high_level_action = np.array([norm_target_alt, norm_target_roll, norm_target_pitch, norm_target_yaw])
        state_goal = np.array([norm_alt, norm_roll, norm_pitch, norm_yaw, 0.0]) # Add placeholder for dt in state
        
        rc_values = self.fc.compute_rc_commands(high_level_action, state_goal, dt)
        
        logger.debug(f"PID In: TgtAlt={norm_target_alt:.2f}, CurAlt={norm_alt:.2f}")
        logger.debug(f"PID Out: RC={rc_values}")
        
        # --- 5. Format and Return RC Command ---
        return {
            "roll": int(rc_values[1]),
            "pitch": int(rc_values[2]),
            "throttle": int(rc_values[0]),
            "yaw": int(rc_values[3]),
            "aux1": 1800, "aux2": 1000 # 1800 = Armed
        }

# --- MQTT Callbacks ---
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        logger.info("Tactical Controller connected to MQTT Broker.")
        client.subscribe(SENSOR_TOPIC)
        client.subscribe(TARGET_TOPIC)
        client.subscribe(STATUS_TOPIC)
        client.subscribe(AI_MODE_TOPIC) 
    else:
        logger.error(f"Failed to connect, return code {reason_code}")

def on_message(client, userdata, msg):
    """Callback to update state and trigger PID compute."""
    controller = userdata["controller"]
    
    try:
        payload = json.loads(msg.payload.decode())
        if msg.topic == TARGET_TOPIC:
            controller.set_normalized_targets(payload)

        elif msg.topic == STATUS_TOPIC:
            controller.is_armed = payload.get("armed", False)
            if not controller.is_armed:
                controller.reset() 

        elif msg.topic == AI_MODE_TOPIC: 
            controller.ai_mode_enabled = payload.get("ai_enabled", False)

            if not controller.ai_mode_enabled:
                controller.reset() 
                logger.info("AI mode disabled, PID controller is now idle.")

        elif msg.topic == SENSOR_TOPIC:
            # This is our main 100Hz control trigger
            command_payload = controller.compute_rc_commands(payload)
            logger.debug(f"Command payload {command_payload}")
            
            if command_payload:
                client.publish(COMMAND_TOPIC, json.dumps(command_payload))

    except json.JSONDecodeError:
        logger.warning(f"Could not decode JSON from topic {msg.topic}")
    except Exception as e:
        logger.error(f"Error in on_message: {e}")

# --- Main ---
def main():
    controller = TacticalControllerWrapper()
    
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="tactical_controller")
    client.user_data_set({"controller": controller})
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except ConnectionRefusedError:
        logger.error("Tactical Controller: Connection to MQTT broker refused.")
        return

    logger.info("Tactical Controller running... (Event-driven)")
    client.loop_forever() 

if __name__ == '__main__':
    logger.setLevel(logging.DEBUG)
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Tactical Controller stopping...")

