import time
import json
import paho.mqtt.client as mqtt
import numpy as np
import tflite_runtime.interpreter as tflite
import logging
from collections import deque
from drone_logging import setup_logger

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
LOG_FILE = "strategic_agent.log"
LOOP_FREQUENCY = 10  # Run the NN at 10Hz
LOOP_TIME = 1.0 / LOOP_FREQUENCY
SMOOTHING_WINDOW_SIZE = 5

# --- Normalization Constants ---
MAX_ALTITUDE = 1.0  # Max altitude in meters for 1.0
MAX_ANGLE = 55.0  # Max roll/pitch in degrees for 1.0
MAX_YAW_ANGLE = 360.0 # Yaw from -180 to 180 -> -1.0 to 1.0 

# --- MQTT Topics ---
SENSOR_TOPIC = "drone/sensors"
STATUS_TOPIC = "drone/status"
AI_MODE_TOPIC = "drone/ai_mode" 
TARGET_TOPIC = "drone/target_setpoints" # Publish NORMALIZED targets here

# --- Setup Logger ---
logger = setup_logger("Strategic_Agent", LOG_FILE)

# --- Global State ---
latest_status = {"armed": False}
ai_mode_enabled = False 

# --- Smoothing Buffers ---
altitude_buffer = deque(maxlen=SMOOTHING_WINDOW_SIZE)
kinematics_buffer = deque(maxlen=SMOOTHING_WINDOW_SIZE)

# --- TFLite Model Setup ---
try:
    interpreter = tflite.Interpreter(model_path="master-model.tflite")
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    logger.info("TFLite model loaded successfully.")
except Exception as e:
    logger.error(f"Failed to load TFLite model: {e}")
    interpreter = None

# --- Helper Functions ---
def normalize_data(raw_altitude, raw_kinematics):
    """Normalizes raw sensor data based on defined maximums."""
    norm_alt = np.clip(raw_altitude / MAX_ALTITUDE, 0.0, 1.0)
    norm_roll = np.clip(raw_kinematics[0] / MAX_ANGLE, -1.0, 1.0)
    norm_pitch = np.clip(raw_kinematics[1] / MAX_ANGLE, -1.0, 1.0)
    norm_yaw = np.clip(raw_kinematics[2] / MAX_YAW_ANGLE, -1.0, 1.0)
    return norm_alt, norm_roll, norm_pitch, norm_yaw

# --- MQTT Callbacks ---
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        logger.info("Strategic Agent connected to MQTT Broker.")
        client.subscribe(SENSOR_TOPIC)
        client.subscribe(STATUS_TOPIC)
        client.subscribe(AI_MODE_TOPIC)
    else:
        logger.error(f"Failed to connect, return code {reason_code}")

def on_message(client, userdata, msg):
    """Callback to update global state and smoothing buffers."""
    global latest_status, altitude_buffer, kinematics_buffer, ai_mode_enabled
    try:
        payload = json.loads(msg.payload.decode())
        
        if msg.topic == SENSOR_TOPIC:
            altitude_buffer.append(payload.get("altitude", 0.0))
            kinematics_buffer.append(payload.get("kinematics", [0, 0, 0]))
            
        elif msg.topic == STATUS_TOPIC:
            latest_status = payload
            if not latest_status.get("armed", False):
                if ai_mode_enabled:
                    logger.warning("Drone DISARMED, forcing AI mode to DISABLED.")
                    ai_mode_enabled = False
                    
        elif msg.topic == AI_MODE_TOPIC: 
            new_ai_state = payload.get("ai_enabled", False)
            if new_ai_state != ai_mode_enabled:
                ai_mode_enabled = new_ai_state
                logger.info(f"AI Mode set to: {'ENABLED' if ai_mode_enabled else 'DISABLED'}")

    except json.JSONDecodeError:
        logger.warning(f"Could not decode JSON from topic {msg.topic}")

def get_smoothed_data():
    """Calculates the average of the data in the buffers."""
    if not altitude_buffer or not kinematics_buffer:
        return None, None 

    smoothed_altitude = sum(altitude_buffer) / len(altitude_buffer)
    smoothed_kinematics = np.mean(np.array(kinematics_buffer), axis=0).tolist()
    
    return smoothed_altitude, smoothed_kinematics

# --- Main Loop ---
def main():
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="strategic_agent")
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except ConnectionRefusedError:
        logger.error("Strategic Agent: Connection to MQTT broker refused.")
        return

    client.loop_start()
    logger.info("Strategic Agent running... (10Hz loop).")

    while True:
        try:
            start_time = time.time()
            
            is_armed = latest_status.get("armed", False)
            raw_altitude, raw_kinematics = get_smoothed_data()

            if ai_mode_enabled and is_armed and raw_altitude is not None and interpreter:
                # --- 1. Normalize Sensor Data for NN Input ---
                norm_alt, norm_roll, norm_pitch, norm_yaw = normalize_data(raw_altitude, raw_kinematics)
                
                # --- 2. Define Strategic Target (Normalized) ---
                # This would come from a higher-level planner, but for now, we'll hardcode it.
                # Let's aim for 1.0 meter altitude (normalized 0.5)
                norm_target_alt_strategic = 1.0 / MAX_ALTITUDE # 0.5
                
                # --- 3. Run NN Inference ---
                # Using your model's input signature from inference-example.py
                input_data = np.array([norm_alt, norm_target_alt_strategic], dtype=np.float32)
                interpreter.set_tensor(input_details[0]['index'], input_data)
                interpreter.invoke()
                # output_data[0] will be the (4,) array you described
                nn_targets = interpreter.get_tensor(output_details[0]['index'])
                # --- 4. Determine NORMALIZED Targets for PID Controller ---
                # *** MODIFIED: Use the NN (4,) output array ***
                
                # Clip all values to be safe
                nn_target_alt_norm = np.clip(nn_targets[0], 0.0, 1.0)
                nn_target_roll_norm = np.clip(nn_targets[1], -1.0, 1.0)
                nn_target_pitch_norm = np.clip(nn_targets[2], -1.0, 1.0)
                nn_target_yaw_norm = np.clip(nn_targets[3], -1.0, 1.0)
                
                target_setpoints = {
                    "target_altitude_norm": float(nn_target_alt_norm),
                    "target_roll_norm": float(nn_target_roll_norm),
                    "target_pitch_norm": float(nn_target_pitch_norm),
                    "target_yaw_norm": float(nn_target_yaw_norm)
                }
                
                # --- 5. Publish NORMALIZED Targets for PID Controller ---
                client.publish(TARGET_TOPIC, json.dumps(target_setpoints))
                logger.info(f"AI Active. Published NORMALIZED Targets: {target_setpoints}")

            else:
                logger.debug(f"AI Inactive. (AI Toggle: {ai_mode_enabled}, Armed: {is_armed})")
            
            # --- Maintain Loop Frequency ---
            elapsed = time.time() - start_time
            if elapsed < LOOP_TIME:
                time.sleep(LOOP_TIME - elapsed)
                
        except KeyboardInterrupt:
            logger.info("Strategic Agent stopping...")
            break
        except Exception as e:
            logger.error(f"Error in Strategic Agent main loop: {e}")
            time.sleep(1)

    client.loop_stop()

if __name__ == '__main__':
    logger.setLevel(logging.DEBUG)
    main()

