import os
import time
import sys
import zmq
import numpy as np
import signal

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from core.shared_memory_manager import SharedMemoryManager
from drone_logging import setup_logger

# Global flag for shutdown
shutting_down = False

def signal_handler(sig, frame):
    global shutting_down
    shutting_down = True

def handle_disarmed(obs: np.ndarray) -> list:
    """
    Mode: Disarmed. Returns dummy RC commands.
    Format: ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
    """
    return [1000, 1000, 900, 1000, 1000, 1000]

def handle_armed(obs: np.ndarray) -> list:
    """
    Mode: Armed. Returns dummy RC commands.
    Format: ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
    """
    return [1000, 1000, 900, 1000, 1800, 1800]

def handle_ai(obs: np.ndarray) -> list:
    """
    Mode: AI. Returns dummy RC commands from 'inference'.
    Format: ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
    """
    # In the future, this will run TFLite inference
    return [1500, 1500, 1500, 1000, 1800, 1800]

def main():
    global shutting_down
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger = setup_logger("drone-inference", "inference.log")
    logger.info("Starting drone-inference service...")

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

    current_mode = "disarmed"
    
    try:
        while not shutting_down:
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
                rc_commands = handle_armed(obs)
            elif current_mode == "ai":
                rc_commands = handle_ai(obs)
            else:
                rc_commands = handle_disarmed(obs)

            # 4. Log observation and RC commands
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
