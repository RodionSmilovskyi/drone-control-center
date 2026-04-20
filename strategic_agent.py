import time
import json
import paho.mqtt.client as mqtt
import numpy as np
import logging
import sys
from collections import deque
from drone_logging import setup_logger

# --- TFLite Model Setup (Optional) ---
try:
    import tflite_runtime.interpreter as tflite
    TFLITE_AVAILABLE = True
except ImportError:
    TFLITE_AVAILABLE = False

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
LOG_FILE = "strategic_agent.log"
LOOP_FREQUENCY = 10 
LOOP_TIME = 1.0 / LOOP_FREQUENCY
SMOOTHING_WINDOW_SIZE = 5

try:
    TARGET_ALTITUDE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
except ValueError:
    TARGET_ALTITUDE = 0.5

# --- Normalization ---
MAX_ALTITUDE = 1.0
MAX_ANGLE = 55.0
MAX_YAW_ANGLE = 360.0 
MAX_XY_SHIFT = 1.0
MAX_VELOCITY = 5.0
FLOW_SCALAR = 0.027

# --- MQTT Topics ---
SENSOR_TOPIC = "drone/sensors"
STATUS_TOPIC = "drone/status"
AI_MODE_TOPIC = "drone/ai_mode" 
TARGET_TOPIC = "drone/target_setpoints"
OBSERVATION_TOPIC = "drone/observation"

# --- Setup Logger ---
logger = setup_logger("Strategic_Agent", LOG_FILE, level=logging.DEBUG)

# --- Global State ---
latest_status = {"armed": False}
ai_mode_enabled = False 
cumulative_shift_x = 0.0
cumulative_shift_y = 0.0
last_sensor_update_time = 0.0

# Buffers
altitude_buffer = deque(maxlen=SMOOTHING_WINDOW_SIZE)
flow_buffer = deque(maxlen=SMOOTHING_WINDOW_SIZE)

def on_connect(client, userdata, flags, rc, properties):
    logger.info(f"MQTT Connected. Code: {rc}")
    client.subscribe([(SENSOR_TOPIC, 0), (STATUS_TOPIC, 0), (AI_MODE_TOPIC, 0)])
    print(f"!!! AGENT CONNECTED TO BROKER !!!")

def on_message(client, userdata, msg):
    global latest_status, ai_mode_enabled, last_sensor_update_time
    global cumulative_shift_x, cumulative_shift_y
    try:
        payload = json.loads(msg.payload.decode())
        if msg.topic == SENSOR_TOPIC:
            last_sensor_update_time = time.time()
            if "altitude" in payload: altitude_buffer.append(payload["altitude"])
            if "flow" in payload:
                f = payload["flow"]
                flow_buffer.append((f.get('x', 0), f.get('y', 0)))
        
        elif msg.topic == STATUS_TOPIC:
            latest_status = payload
            
        elif msg.topic == AI_MODE_TOPIC:
            new_state = payload.get("ai_enabled", False)
            if new_state != ai_mode_enabled:
                ai_mode_enabled = new_state
                if ai_mode_enabled:
                    cumulative_shift_x, cumulative_shift_y = 0.0, 0.0
                    logger.info("AI Enabled - Shift Reset")
    except Exception as e:
        logger.error(f"MQTT Msg Error: {e}")

def get_smoothed_data():
    now = time.time()
    if now - last_sensor_update_time > 2.0:
        return None, None, f"STALE ({now - last_sensor_update_time:.1f}s)"
    if not altitude_buffer:
        return None, None, "EMPTY_BUFFER"
    
    avg_alt = sum(altitude_buffer) / len(altitude_buffer)
    avg_flow = np.mean(list(flow_buffer), axis=0).tolist() if flow_buffer else [0, 0]
    return avg_alt, avg_flow, "OK"

from core.shared_memory_manager import SharedMemoryManager

def main():
    global cumulative_shift_x, cumulative_shift_y, last_heartbeat
    last_time = time.time()
    last_heartbeat = 0
    
    shm_name = "drone_sensor_data"
    shm_size = 6 * 8
    shm_mgr = None

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"CRITICAL: MQTT Connection Failed: {e}")
        return

    client.loop_start()

    while True:
        try:
            loop_start = time.time()
            dt = loop_start - last_time
            last_time = loop_start

            # 1. Read from SHM instead of buffers
            if shm_mgr is None:
                try:
                    shm_mgr = SharedMemoryManager(shm_name, shm_size, create=False)
                except:
                    shm_mgr = None
            
            # Values from SHM: [alt, sx, sy, vx_norm, vy_norm, heartbeat]
            alt, sx, sy, vx_norm, vy_norm = None, None, None, None, None
            status_msg = "DISCONNECTED"
            if shm_mgr:
                try:
                    data = shm_mgr.read_array(np.float64, (6,))
                    heartbeat = data[5]
                    if (time.time() - heartbeat) < 1.0:
                        alt, sx, sy, vx_norm, vy_norm = data[0], data[1], data[2], data[3], data[4]
                        status_msg = "OK"
                    else:
                        status_msg = "STALE"
                except:
                    shm_mgr.close()
                    shm_mgr = None
            
            armed = latest_status.get("armed", False)

            # Heartbeat (once per second)
            if loop_start - last_heartbeat > 1.0:
                print(f"HEARTBEAT | AI:{ai_mode_enabled} | Armed:{armed} | Sensors:{status_msg} | Alt:{alt}")
                last_heartbeat = loop_start

            if ai_mode_enabled and armed and alt is not None:
                # Observation (7 elements) - Using values directly from SHM
                obs = [
                    np.clip(alt / MAX_ALTITUDE, 0, 1),
                    np.clip(sx / MAX_XY_SHIFT, -1, 1),
                    np.clip(sy / MAX_XY_SHIFT, -1, 1),
                    np.clip(vx_norm, -1, 1),
                    np.clip(vy_norm, -1, 1),
                    TARGET_ALTITUDE / MAX_ALTITUDE,
                    time.time() % 60 
                ]

                # Actions
                targets = {
                    "target_altitude_norm": round(obs[5], 2),
                    "target_roll_norm": 0.0,
                    "target_pitch_norm": 0.0,
                    "target_yaw_norm": 0.0
                }

                client.publish(TARGET_TOPIC, json.dumps(targets))
                client.publish(OBSERVATION_TOPIC, json.dumps({"observation": [round(float(x), 4) for x in obs]}))
            
            elapsed = time.time() - loop_start
            if elapsed < LOOP_TIME:
                time.sleep(LOOP_TIME - elapsed)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"LOOP ERROR: {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()
