import multiprocessing
from multiprocessing import shared_memory
import numpy as np
from typing import Optional, Any

class SharedMemoryManager:
    """
    Manages shared memory using the multiprocessing package.
    Ensures synchronized access to prevent data tearing.
    """
    def __init__(self, name: str, size: int, create: bool = False, lock: Optional[multiprocessing.Lock] = None):
        self.name = name
        self.size = size
        # If no lock is provided, it creates a new one.
        # Note: For multi-process use, the same lock object must be shared between processes.
        self.lock = lock or multiprocessing.Lock()
        
        if create:
            try:
                self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
            except FileExistsError:
                self.shm = shared_memory.SharedMemory(name=name)
        else:
            self.shm = shared_memory.SharedMemory(name=name)

    def write_bytes(self, data: bytes):
        """Write raw bytes to shared memory with locking."""
        with self.lock:
            length = min(len(data), self.size)
            self.shm.buf[:length] = data[:length]

    def read_bytes(self, n_bytes: Optional[int] = None) -> bytes:
        """Read raw bytes from shared memory with locking."""
        with self.lock:
            n = n_bytes if n_bytes is not None else self.size
            return bytes(self.shm.buf[:n])

    def write_array(self, data: np.ndarray):
        """Write a numpy array to shared memory with locking."""
        with self.lock:
            shm_array = np.ndarray(data.shape, dtype=data.dtype, buffer=self.shm.buf)
            shm_array[:] = data[:]

    def read_array(self, dtype: Any, shape: tuple) -> np.ndarray:
        """Read a numpy array from shared memory with locking (returns a copy)."""
        with self.lock:
            shm_array = np.ndarray(shape, dtype=dtype, buffer=self.shm.buf)
            return shm_array.copy()

    def close(self):
        """Close the shared memory connection for this instance."""
        if hasattr(self, 'shm'):
            self.shm.close()

    def unlink(self):
        """Remove the shared memory from the system. Call only once from the owner process."""
        if hasattr(self, 'shm'):
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
