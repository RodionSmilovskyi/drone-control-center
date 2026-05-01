import os
import time
import sys
import signal
import numpy as np

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from core.shared_memory_manager import SharedMemoryManager

DRONE_ENV = os.environ.get("DRONE_ENV", "PI")

print(f"DEBUG: DRONE_ENV is '{DRONE_ENV}'")

if DRONE_ENV == "WSL":
    from sensor_mock import SensorMock as SensorProvider
else:
    try:
        from sensor_real import SensorReal as SensorProvider
    except ImportError as e:
        print(f"[CRITICAL] Failed to load real sensors on PI: {e}")
        print("Falling back to zero-data dummy class.")
        class SensorReal:
            def read(self): return [0.0] * 5
        SensorProvider = SensorReal

def main():
    provider = SensorProvider()
    
    # SHM structure: [alt, sx, sy, vx, vy, heartbeat] (6 floats)
    shm_size = 6 * 8 
    shm_name = "drone_sensor_data"

    hb_shm_name = "system_heartbeats"
    hb_shm_size = 3 * 8
    
    # Pre-emptive cleanup using the manager's logic or simple try-block
    shm_mgr = None
    hb_shm_mgr = None
    
    try:
        shm_mgr = SharedMemoryManager(shm_name, shm_size, create=True)
    except FileExistsError:
        # If it exists but wasn't unlinked, try to attach and unlink first
        temp_mgr = SharedMemoryManager(shm_name, shm_size, create=False)
        temp_mgr.unlink()
        shm_mgr = SharedMemoryManager(shm_name, shm_size, create=True)

    try:
        hb_shm_mgr = SharedMemoryManager(hb_shm_name, hb_shm_size, create=True)
    except FileExistsError:
        # If it exists but wasn't unlinked, try to attach and unlink first
        temp_mgr = SharedMemoryManager(hb_shm_name, hb_shm_size, create=False)
        temp_mgr.unlink()
        hb_shm_mgr = SharedMemoryManager(hb_shm_name, hb_shm_size, create=True)
    
    def shutdown_handler(signum, frame):
        print(f"Caught signal {signum}, shutting down...")
        if shm_mgr:
            shm_mgr.close()
            shm_mgr.unlink()
        if hb_shm_mgr:
            hb_shm_mgr.close()
            hb_shm_mgr.unlink()
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)

    print(f"Sensor service started in {DRONE_ENV} mode. Writing to SHM: {shm_name}")
    try:
        while True:
            # provider.read() returns [altitude, shift_x, shift_y, vel_x, vel_y] (5 values)
            raw_data = provider.read()
            
            # Pack into SHM: [alt, sx, sy, vx, vy, heartbeat]
            shm_data = [
                raw_data[0], # alt
                raw_data[1], # sx
                raw_data[2], # sy
                raw_data[3], # vx
                raw_data[4], # vy
                time.time()  # heartbeat
            ]
            
            array_data = np.array(shm_data, dtype=np.float64)
            shm_mgr.write_array(array_data)

            # Update system heartbeats (Index 0 for sensors)
            try:
                hb_shm_mgr.write_array_index(0, time.time(), np.float64, (3,))
            except Exception as e:
                print(f"HB Error: {e}")
            
            time.sleep(1/30.0) # 30Hz
    except Exception as e:
        print(f"Error in sensor loop: {e}")
    finally:
        if shm_mgr:
            shm_mgr.close()
            shm_mgr.unlink()
        if hb_shm_mgr:
            hb_shm_mgr.close()
            hb_shm_mgr.unlink()

if __name__ == "__main__":
    main()
