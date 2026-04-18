import sys
import os
import time
import numpy as np

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.shared_memory_manager import SharedMemoryManager

def main():
    shm_name = "drone_sensor_data"
    shm_size = 6 * 8
    
    # Values: [altitude, shift_x, shift_y, velocity_x, velocity_y]
    test_data = [
        [0.5, 0.1, -0.1, 0.05, -0.05],
        [0.8, 0.5, 0.5, 0.2, 0.2],
        [0.2, -0.5, -0.5, -0.2, -0.2],
        [1.0, 1.0, 1.0, 1.0, 1.0],
        [0.0, -1.0, -1.0, -1.0, -1.0]
    ]

    print(f"Injecting test data into {shm_name}...")
    with SharedMemoryManager(shm_name, shm_size, create=True) as shm_mgr:
        try:
            for data in test_data:
                # Add heartbeat
                full_data = data + [time.time()]
                print(f"Writing: {full_data}")
                array_data = np.array(full_data, dtype=np.float64)
                shm_mgr.write_array(array_data)
                time.sleep(2) # 2 seconds per frame for visibility
        except KeyboardInterrupt:
            pass
        finally:
            shm_mgr.unlink()
    print("Test injection complete.")

if __name__ == "__main__":
    main()
