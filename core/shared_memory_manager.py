import multiprocessing
from multiprocessing import shared_memory
from multiprocessing import resource_tracker
import numpy as np
from typing import Optional, Any

def unregister_shm(name: str):
    """
    Directly unregister a shared memory segment from the resource tracker.
    This prevents the tracker from unlinking the segment when this process exits.
    """
    try:
        resource_tracker.unregister(f"/{name}" if not name.startswith("/") else name, "shared_memory")
    except Exception:
        # Fallback for different OS/Python versions
        try:
            resource_tracker.unregister(name, "shared_memory")
        except Exception:
            pass

class SharedMemoryManager:
    """
    Manages shared memory using the multiprocessing package.
    Ensures synchronized access to prevent data tearing.
    """
    def __init__(self, name: str, size: int, create: bool = False, lock: Optional[Any] = None):
        self.name = name
        self.size = size
        self.lock = lock
        
        if create:
            try:
                self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
            except FileExistsError:
                self.shm = shared_memory.SharedMemory(name=name)
        else:
            self.shm = shared_memory.SharedMemory(name=name)
        
        # IMMEDIATELY unregister so the resource_tracker doesn't kill it on exit
        unregister_shm(self.shm.name)

    def write_bytes(self, data: bytes):
        if self.lock:
            with self.lock:
                self._write_bytes(data)
        else:
            self._write_bytes(data)

    def _write_bytes(self, data: bytes):
        length = min(len(data), self.size)
        self.shm.buf[:length] = data[:length]

    def read_bytes(self, n_bytes: Optional[int] = None) -> bytes:
        if self.lock:
            with self.lock:
                return self._read_bytes(n_bytes)
        return self._read_bytes(n_bytes)

    def _read_bytes(self, n_bytes: Optional[int] = None) -> bytes:
        n = n_bytes if n_bytes is not None else self.size
        return bytes(self.shm.buf[:n])

    def write_array(self, data: np.ndarray):
        if self.lock:
            with self.lock:
                self._write_array(data)
        else:
            self._write_array(data)

    def _write_array(self, data: np.ndarray):
        shm_array = np.ndarray(data.shape, dtype=data.dtype, buffer=self.shm.buf)
        shm_array[:] = data[:]

    def read_array(self, dtype: Any, shape: tuple) -> np.ndarray:
        if self.lock:
            with self.lock:
                return self._read_array(dtype, shape)
        return self._read_array(dtype, shape)

    def _read_array(self, dtype: Any, shape: tuple) -> np.ndarray:
        shm_array = np.ndarray(shape, dtype=dtype, buffer=self.shm.buf)
        return shm_array.copy()

    def close(self):
        """Close the shared memory connection for this instance."""
        if hasattr(self, 'shm') and self.shm:
            try:
                self.shm.close()
            except Exception:
                pass

    def unlink(self):
        """Remove the shared memory from the system. Call only once from the owner process."""
        if hasattr(self, 'shm') and self.shm:
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
