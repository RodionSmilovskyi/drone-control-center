import os
import time
import sys
import signal
import numpy as np
import zmq
import logging

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from core.shared_memory_manager import SharedMemoryManager
from drone_logging import setup_logger

DRONE_ENV = os.environ.get("DRONE_ENV", "PI")

# Determine which provider to use
if DRONE_ENV == "WSL":
    from fc_mock import FCMock as FCProvider
else:
    try:
        from fc_real import FCReal as FCProvider
    except (ImportError, Exception) as e:
        print(f"[CRITICAL] Failed to load real FC on PI: {e}")
        print("Falling back to dummy mock provider.")
        from fc_mock import FCMock as FCProvider

def main():
    logger = setup_logger("drone-fc", "fc.log")
    logger.info(f"Starting drone-fc service in {DRONE_ENV} mode...")

    try:
        provider = FCProvider()
    except Exception as e:
        logger.error(f"Failed to initialize FC provider: {e}")
        sys.exit(1)
    
    # Heartbeat Setup (Index 2 for FC)
    hb_shm_name = "system_heartbeats"
    hb_shm_size = 3 * 8
    hb_shm_mgr = None
    
    # ZMQ Setup to receive RC commands from drone-inference
    context = zmq.Context()
    rc_sub = context.socket(zmq.SUB)
    rc_sub.setsockopt(zmq.CONFLATE, 1)  # Always get the latest command
    rc_sub.connect("tcp://127.0.0.1:5556")
    rc_sub.setsockopt_string(zmq.SUBSCRIBE, "")

    def shutdown_handler(signum, frame):
        logger.info(f"Caught signal {signum}, shutting down...")
        if hasattr(provider, 'close'):
            provider.close()
        if hb_shm_mgr:
            hb_shm_mgr.close()
        rc_sub.close()
        context.term()
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)

    logger.info("drone-fc service ready. Listening for commands on ZMQ 5556.")
    
    # Default safe commands: [roll, pitch, throttle, yaw, aux1, aux2]
    # throttle 900, others 1500, aux1 1000 (disarmed)
    rc_commands = [1500, 1500, 900, 1500, 1000, 1000]

    try:
        while True:
            # 1. Non-blocking read of the latest RC commands
            try:
                new_rc = rc_sub.recv_pyobj(flags=zmq.NOBLOCK)
                if new_rc and len(new_rc) == 6:
                    rc_commands = new_rc
            except zmq.Again:
                pass
            except Exception as e:
                logger.error(f"ZMQ Receive Error: {e}")

            # 2. Send commands to the FC provider
            try:
                provider.send_rc(rc_commands)
            except Exception as e:
                logger.error(f"Provider send_rc error: {e}")

            # 3. Update heartbeat (Index 2)
            if hb_shm_mgr is None:
                try:
                    hb_shm_mgr = SharedMemoryManager(hb_shm_name, hb_shm_size, create=False)
                except Exception:
                    pass
            # Update heartbeat (Index 2)
            if hb_shm_mgr:
                try:
                    hb_shm_mgr.write_array_index(2, time.time(), np.float64, (3,))
                except Exception as e:
                    logger.debug(f"Heartbeat write error: {e}")
                    hb_shm_mgr.close()
                    hb_shm_mgr = None
            
            # Run at 100Hz
            time.sleep(1/100.0)
            
    except Exception as e:
        logger.error(f"Unexpected error in main loop: {e}")
    finally:
        if hb_shm_mgr:
            hb_shm_mgr.close()
        rc_sub.close()
        context.term()

if __name__ == "__main__":
    main()
