import os
import time
import sys
import zmq
import numpy as np
import signal
import argparse
import logging

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from core.shared_memory_manager import SharedMemoryManager
from drone_logging import setup_logger
from flight_controller import FlightController

# Global flag for shutdown
shutting_down = False

def signal_handler(sig, frame):
    global shutting_down
    shutting_down = True

def handle_disarmed(obs: np.ndarray, fc: FlightController) -> list:
    """
    Mode: Disarmed. Returns dummy RC commands and resets FC.
    Format: ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
    """
    fc.reset()
    return [1000, 1000, 900, 1000, 1000, 1000]

def handle_armed(obs: np.ndarray, fc: FlightController) -> list:
    """
    Mode: Armed. Returns dummy RC commands and resets FC.
    Format: ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
    """
    fc.reset()
    return [1000, 1000, 900, 1000, 1800, 1800]

def handle_ai(obs: np.ndarray, fc: FlightController) -> list:
    """
    Mode: AI. Returns RC commands from FlightController with hardcoded action.
    Format: ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
    """
    # Hardcoded high-level action: [desired_alt, roll, pitch, yaw_rate]
    # action[0]=0.3 maps to desired_alt_norm = (0.3+1)/2 = 0.65
    high_level_action = np.array([0.3, 0.0, 0.0, 0.0], dtype=np.float32)
    
    # current altitude is at obs[0]
    current_alt_norm = obs[0]
    
    # Compute low-level RC commands (4 channels: throttle, roll, pitch, yaw)
    # Using dt=0.1 because the loop runs at 10Hz (time.sleep(0.1))
    low_level_rc = fc.compute_rc_commands(high_level_action, current_alt_norm, dt=0.1)
    
    # reorder to [roll, pitch, throttle, yaw, aux1, aux2]
    rc_throttle, rc_roll, rc_pitch, rc_yaw = low_level_rc.tolist()
    
    return [int(rc_roll), int(rc_pitch), int(rc_throttle), int(rc_yaw), 1800, 1800]

def main():
    global shutting_down
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-log", action="store_true", help="Disable periodic logging")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger = setup_logger("drone-inference", "inference.log")
    logger.info("Starting drone-inference service...")

    # Initialize Flight Controller
    fc = FlightController()

    # ZMQ Setup
    context = zmq.Context()
    
    # SUB to dashboard for modes
    mode_sub = context.socket(zmq.SUB)
    mode_sub.connect("tcp://127.0.0.1:5555")
    mode_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    
    # PUB for RC commands
    rc_pub = context.socket(zmq.PUB)
    rc_pub.bind("tcp://127.0.0.1:5556")

    # Shared Memory for observations
    shm_name = "drone_sensor_data"
    shm_size = 6 * 8  # 6 doubles
    shm_mgr = None

    # Heartbeat setup
    hb_shm_name = "system_heartbeats"
    hb_shm_size = 3 * 8
    hb_shm_mgr = None

    current_mode = "disarmed"
    
    try:
        while not shutting_down:
            # 0. Update Heartbeat (Index 1 for Inference)
            if hb_shm_mgr is None:
                try:
                    hb_shm_mgr = SharedMemoryManager(hb_shm_name, hb_shm_size, create=False)
                except Exception:
                    hb_shm_mgr = None
            
            if hb_shm_mgr:
                try:
                    hbs = hb_shm_mgr.read_array(np.float64, (3,))
                    hbs[1] = time.time()
                    hb_shm_mgr.write_array(hbs)
                except Exception:
                    hb_shm_mgr.close()
                    hb_shm_mgr = None

            # 1. Non-blocking read mode
            try:
                new_mode = mode_sub.recv_string(flags=zmq.NOBLOCK)
                if new_mode:
                    current_mode = new_mode
                    logger.info(f"Mode changed to: {current_mode}")
            except zmq.Again:
                pass

            # 2. Read observation from Shared Memory
            obs = np.zeros(6, dtype=np.float64)
            if shm_mgr is None:
                try:
                    shm_mgr = SharedMemoryManager(shm_name, shm_size, create=False)
                except Exception:
                    shm_mgr = None
            
            if shm_mgr:
                try:
                    obs = shm_mgr.read_array(np.float64, (6,))
                except Exception:
                    shm_mgr.close()
                    shm_mgr = None

            # 3. Process based on mode
            if current_mode == "armed":
                rc_commands = handle_armed(obs, fc)
            elif current_mode == "ai":
                rc_commands = handle_ai(obs, fc)
            else:
                rc_commands = handle_disarmed(obs, fc)

            # 4. Log observation and RC commands
            if not args.no_log and logger.isEnabledFor(logging.INFO):
                logger.info(f"OBS: {obs.tolist()} | RC: {rc_commands}")

            # 5. Publish RC commands
            rc_pub.send_pyobj(rc_commands)

            time.sleep(0.1)  # 10Hz loop
    except Exception as e:
        logger.error(f"Inference error: {e}")
    finally:
        if shm_mgr:
            shm_mgr.close()
        rc_pub.close()
        mode_sub.close()
        context.term()
        logger.info("Inference service shut down.")

if __name__ == "__main__":
    main()
