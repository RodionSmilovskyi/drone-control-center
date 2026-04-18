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
    except ImportError:
        class SensorReal:
            def read(self): return [0.0] * 5
        SensorProvider = SensorReal

def main():
    provider = SensorProvider()
    
    shm_size = 6 * 8 
    shm_name = "drone_sensor_data"
    
    # Pre-emptive cleanup to avoid FileExistsError or stale segments
    try:
        # Use a raw shared_memory check for cleanup to avoid complexity
        from multiprocessing import shared_memory
        existing = shared_memory.SharedMemory(name=shm_name)
        existing.close()
        existing.unlink()
        print(f"Cleaned up stale SHM: {shm_name}")
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"Cleanup note: {e}")

    shm_mgr = SharedMemoryManager(shm_name, shm_size, create=True)
    
    def shutdown_handler(signum, frame):
        print(f"Caught signal {signum}, shutting down...")
        shm_mgr.close()
        shm_mgr.unlink()
        sys.exit(0)

    # Register signals for graceful shutdown (SIGTERM for systemd, SIGINT for Ctrl+C)
    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)

    print(f"Sensor service started in {DRONE_ENV} mode. Writing to SHM: {shm_name}")
    try:
        while True:
            data = provider.read()
            # Heartbeat at index 5
            data.append(time.time())
            array_data = np.array(data, dtype=np.float64)
            shm_mgr.write_array(array_data)
            time.sleep(1/30.0) # 30Hz
    except Exception as e:
        print(f"Error in sensor loop: {e}")
    finally:
        shm_mgr.close()
        shm_mgr.unlink()

if __name__ == "__main__":
    main()
