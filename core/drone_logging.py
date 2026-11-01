import atexit
import logging
import queue
import sys
import os
from logging.handlers import WatchedFileHandler, QueueHandler, QueueListener


class _NonBlockingQueueHandler(QueueHandler):
    """QueueHandler that never blocks the real-time flight loop if the bounded log queue is full."""
    def enqueue(self, record):
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(record)
            except queue.Full:
                pass


def setup_logger(name, log_file, level=None):
    """
    Creates and configures a logger that writes to both a file
    and the console (stdout) asynchronously via a bounded QueueListener
    so SD card writeback or stdout stalls never block the flight loop.
    If DRONE_QUIET environment variable is set to '1', level defaults to WARNING.
    """
    if level is None:
        if os.environ.get("DRONE_QUIET") == "1":
            level = logging.WARNING
        else:
            level = logging.INFO

    # Create a logger
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Create a formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    # Resolve log path to project root logs/ directory if not an absolute path
    if not os.path.isabs(log_file):
        # Since this module is located in core/, project_root is one level up
        core_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(core_dir)
        logs_dir = os.path.join(project_root, "logs")
        os.makedirs(logs_dir, exist_ok=True)
        log_file = os.path.join(logs_dir, log_file)
    else:
        os.makedirs(os.path.dirname(log_file), exist_ok=True)

    # Add non-blocking queue handler and background listener *only if* logger doesn't have handlers
    if not logger.hasHandlers():
        file_handler = WatchedFileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)

        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)

        log_queue = queue.Queue(maxsize=512)
        queue_handler = _NonBlockingQueueHandler(log_queue)
        queue_handler.setLevel(level)
        logger.addHandler(queue_handler)

        listener = QueueListener(log_queue, file_handler, console_handler, respect_handler_level=True)
        listener.start()
        atexit.register(listener.stop)
    
    # Propagate to root logger
    logger.propagate = False 

    return logger
