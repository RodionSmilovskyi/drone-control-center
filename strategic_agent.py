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
MAX_XY_SHIFT = 1.0  # Meters
MAX_VELOCITY = 5.0  # Meters per second
FLOW_SCALAR = 0.18 # Tunable constant for pixel-to-metric conversion (guessed value)

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
cumulative_shift_x = 0.0
cumulative_shift_y = 0.0
last_time = time.time()

# --- Smoothing Buffers ---
# These buffers act as our "Short Term Memory" to fuse data from different sources
altitude_buffer = deque(maxlen=SMOOTHING_WINDOW_SIZE)
kinematics_buffer = deque(maxlen=SMOOTHING_WINDOW_SIZE)
flow_buffer = deque(maxlen=SMOOTHING_WINDOW_SIZE) # Added for flow
obstacle_buffer = deque(maxlen=SMOOTHING_WINDOW_SIZE) # Added for obstacle

# --- TFLite Model Setup ---
# try:
#     interpreter = tflite.Interpreter(model_path="master-model.tflite")
#     interpreter.allocate_tensors()
#     input_details = interpreter.get_input_details()
#     output_details = interpreter.get_output_details()
#     logger.info("TFLite model loaded successfully.")
# except Exception as e:
#     logger.error(f"Failed to load TFLite model: {e}")
#     interpreter = None
interpreter = None # Disabled for now

# --- Helper Functions ---
def normalize_data(raw_altitude, raw_kinematics):
    """Normalizes raw sensor data based on defined maximums."""
    norm_alt = np.clip(raw_altitude / MAX_ALTITUDE, 0.0, 1.0)
    norm_roll = np.clip(raw_kinematics[0] / MAX_ANGLE, -1.0, 1.0)
    norm_pitch = np.clip(raw_kinematics[1] / MAX_ANGLE, -1.0, 1.0)
    norm_yaw = np.clip(raw_kinematics[2] / MAX_YAW_ANGLE, -1.0, 1.0)
    return norm_alt, norm_roll, norm_pitch, norm_yaw

def get_dummy_action(observation):
    """Returns [norm_target_alt_strategic * 2 - 1, 0, 0, 0] as requested."""
    norm_target_alt_strategic = observation[5] # goal_altitude is at index 5
    return [norm_target_alt_strategic * 2 - 1, 0.0, 0.0, 0.0]

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
            # --- DATA FUSION LOGIC ---
            # We check WHICH data is in the packet and update only that buffer.
            # This allows fc_interface (kinematics) and sensor.py (altitude/flow)
            # to publish asynchronously without overwriting each other with zeros.
            
            if "altitude" in payload:
                altitude_buffer.append(payload["altitude"])
            
            if "kinematics" in payload:
                kinematics_buffer.append(payload["kinematics"])
                
            if "obstacle_distance" in payload:
                obstacle_buffer.append(payload["obstacle_distance"])
                
            if "flow" in payload:
                # payload["flow"] is expected to be {'x': dx, 'y': dy}
                flow_data = payload["flow"]
                flow_buffer.append((flow_data.get('x', 0), flow_data.get('y', 0)))
            
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
    # We need altitude to run. Flow might be zero if stationary.
    if not altitude_buffer:
        return None, None 

    smoothed_altitude = sum(altitude_buffer) / len(altitude_buffer)
    
    if flow_buffer:
        flow_array = np.array(flow_buffer)
        smoothed_flow = np.mean(flow_array, axis=0).tolist()
    else:
        smoothed_flow = [0, 0]
    
    return smoothed_altitude, smoothed_flow

# --- Main Loop ---
def main():
    global cumulative_shift_x, cumulative_shift_y, last_time
    
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
            dt = start_time - last_time
            last_time = start_time
            
            is_armed = latest_status.get("armed", False)
            
            # This function retrieves the FUSED state from our buffers
            smoothed_altitude, smoothed_flow = get_smoothed_data()

            if ai_mode_enabled and is_armed and smoothed_altitude is not None:
                # --- 1. Calculate Velocity and Shift from Optical Flow ---
                # Formula: v = (flow * altitude * FLOW_SCALAR) / dt
                # We use FLOW_SCALAR to map pixels/frame to something physical
                if dt > 0:
                    velocity_x = (smoothed_flow[0] * smoothed_altitude * FLOW_SCALAR) / dt
                    velocity_y = (smoothed_flow[1] * smoothed_altitude * FLOW_SCALAR) / dt
                else:
                    velocity_x, velocity_y = 0.0, 0.0

                cumulative_shift_x += velocity_x * dt
                cumulative_shift_y += velocity_y * dt

                # --- 2. Construct Observation ---
                # [altitude, shift_x, shift_y, velocity_x, velocity_y, goal_alt]
                norm_alt = np.clip(smoothed_altitude / MAX_ALTITUDE, 0.0, 1.0)
                norm_shift_x = np.clip(cumulative_shift_x / MAX_XY_SHIFT, -1.0, 1.0)
                norm_shift_y = np.clip(cumulative_shift_y / MAX_XY_SHIFT, -1.0, 1.0)
                norm_velocity_x = np.clip(velocity_x / MAX_VELOCITY, -1.0, 1.0)
                norm_velocity_y = np.clip(velocity_y / MAX_VELOCITY, -1.0, 1.0)
                
                # norm_target_alt_strategic = 1.0 / MAX_ALTITUDE # 0.5
                norm_target_alt_strategic = 0.1 / MAX_ALTITUDE
                
                observation = [
                    norm_alt, 
                    norm_shift_x, 
                    norm_shift_y, 
                    norm_velocity_x, 
                    norm_velocity_y, 
                    norm_target_alt_strategic
                ]
                
                # --- 3. Action Generation (Dummy) ---
                action = get_dummy_action(observation)
                
                # Map back to payload. Dummy action returns [alt, roll, pitch, yaw] in [-1, 1]
                # Tactical controller expects target_altitude_norm in [0, 1]
                target_setpoints = {
                    "target_altitude_norm": round(float(norm_target_alt_strategic), 2),
                    "target_roll_norm": round(float(action[1]), 2),
                    "target_pitch_norm": round(float(action[2]), 2),
                    "target_yaw_norm": round(float(action[3]), 2)
                }
                
                # --- 4. Publish Targets ---
                client.publish(TARGET_TOPIC, json.dumps(target_setpoints))
                
                # Detailed Debug Logging
                logger.info(f"AI ACTIVE | Alt: {smoothed_altitude:.2f}m | Flow: {smoothed_flow} | Vel: ({velocity_x:.2f}, {velocity_y:.2f}) | Shift: ({cumulative_shift_x:.2f}, {cumulative_shift_y:.2f})")
                logger.info(f"OBSERVATION: {[round(x, 3) for x in observation]}")
                logger.debug(f"PUBLISHED TARGETS: {target_setpoints}")

            else:
                if ai_mode_enabled and smoothed_altitude is None:
                    logger.warning("AI Enabled but sensors missing! Waiting for data fusion...")
                else:
                    logger.debug(f"AI Inactive. (AI Toggle: {ai_mode_enabled}, Armed: {is_armed})")
                    # Reset accumulators when inactive
                    cumulative_shift_x = 0.0
                    cumulative_shift_y = 0.0
            
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