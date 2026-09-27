import unittest
import multiprocessing
import numpy as np
import time
import os
import sys

# Add project root to sys.path to allow importing from core
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.shared_memory_manager import SharedMemoryManager

def stress_writer(name, size, val, lock, stop_event):
    """Writes a consistent value to the shared memory repeatedly."""
    mgr = SharedMemoryManager(name, size, create=False, lock=lock)
    data = np.full((size // 4,), val, dtype=np.int32)
    while not stop_event.is_set():
        mgr.write_array(data)
    mgr.close()

def stress_reader(name, size, lock, stop_event, error_counter):
    """Reads from shared memory and checks if data is consistent (no tearing)."""
    mgr = SharedMemoryManager(name, size, create=False, lock=lock)
    while not stop_event.is_set():
        data = mgr.read_array(np.int32, (size // 4,))
        if len(data) > 0 and not np.all(data == data[0]):
            with error_counter.get_lock():
                error_counter.value += 1
    mgr.close()

class TestSharedMemoryManager(unittest.TestCase):
    def setUp(self):
        self.shm_name = "test_shm_manager"
        self.shm_size = 4096  # 4KB
        self.lock = multiprocessing.Lock()
        # Ensure any leftover shm is cleaned up
        try:
            temp_shm = multiprocessing.shared_memory.SharedMemory(name=self.shm_name)
            temp_shm.close()
            temp_shm.unlink()
        except FileNotFoundError:
            pass
            
        self.mgr = SharedMemoryManager(self.shm_name, self.shm_size, create=True, lock=self.lock)

    def tearDown(self):
        self.mgr.close()
        self.mgr.unlink()

    def test_basic_read_write_bytes(self):
        test_data = b"drone sensor data"
        self.mgr.write_bytes(test_data)
        read_data = self.mgr.read_bytes(len(test_data))
        self.assertEqual(read_data, test_data)

    def test_numpy_integration(self):
        # Create a 2D array
        test_array = np.random.rand(10, 10).astype(np.float32)
        self.mgr.write_array(test_array)
        read_array = self.mgr.read_array(np.float32, (10, 10))
        np.testing.assert_array_almost_equal(test_array, read_array)

    def test_data_tearing_prevention(self):
        """
        Tests if the lock prevents data tearing by having multiple processes
        write different patterns while a reader checks for consistency.
        """
        stop_event = multiprocessing.Event()
        error_counter = multiprocessing.Value('i', 0)
        
        # Two writers writing different values (all 1s or all 2s)
        p1 = multiprocessing.Process(target=stress_writer, args=(self.shm_name, self.shm_size, 1, self.lock, stop_event))
        p2 = multiprocessing.Process(target=stress_writer, args=(self.shm_name, self.shm_size, 2, self.lock, stop_event))
        
        # One reader checking for tearing
        reader = multiprocessing.Process(target=stress_reader, args=(self.shm_name, self.shm_size, self.lock, stop_event, error_counter))
        
        p1.start()
        p2.start()
        reader.start()
        
        # Let them run for a short duration
        time.sleep(1)
        
        stop_event.set()
        p1.join()
        p2.join()
        reader.join()
        
        self.assertEqual(error_counter.value, 0, f"Detected {error_counter.value} instances of data tearing!")

if __name__ == "__main__":
    unittest.main()
