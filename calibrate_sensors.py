import time
import numpy as np
import sys
import os

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
from core.shared_memory_manager import SharedMemoryManager

def main():
    shm_name = "drone_sensor_data"
    shm_size = 6 * 8
    
    try:
        shm_mgr = SharedMemoryManager(shm_name, shm_size, create=False)
    except Exception as e:
        print(f"Error: Could not connect to shared memory. Is the sensor service running? ({e})")
        return

    print("--- Sensor Calibration Tool ---")
    print("This tool shows raw integrated shifts from the sensor service.")
    print("1. Place drone at desired height.")
    print("2. Press ENTER to capture STARTING position.")
    print("3. Move drone exactly 10cm (or any known distance).")
    print("4. Press ENTER to capture ENDING position.")
    print("Ctrl+C to exit.\n")

    try:
        while True:
            input(">>> Press ENTER to capture START...")
            data = shm_mgr.read_array(np.float64, (6,))
            start_alt = data[0]
            start_sx = data[1]
            start_sy = data[2]
            print(f"START: Alt={start_alt:.3f}m | X={start_sx:.3f} | Y={start_sy:.3f}")

            input(">>> Move drone, then press ENTER to capture END...")
            data = shm_mgr.read_array(np.float64, (6,))
            end_alt = data[0]
            end_sx = data[1]
            end_sy = data[2]
            
            dx = end_sx - start_sx
            dy = end_sy - start_sy
            dist = np.sqrt(dx**2 + dy**2)
            
            print(f"END:   Alt={end_alt:.3f}m | X={end_sx:.3f} | Y={end_sy:.3f}")
            print(f"--- RESULTS ---")
            print(f"Delta X: {dx:.4f}")
            print(f"Delta Y: {dy:.4f}")
            print(f"Calculated Magnitude: {dist:.4f}")
            print(f"Avg Altitude: {(start_alt + end_alt)/2:.3f}m")
            print("---------------\n")

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        shm_mgr.close()

if __name__ == "__main__":
    main()
