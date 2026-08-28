import logging
import sys
import os
from logging.handlers import WatchedFileHandler

def setup_logger(name, log_file, level=None):
    """
    Creates and configures a logger that writes to both a file
    and the console (stdout).
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
        project_root = os.path.dirname(os.path.abspath(__file__))
        logs_dir = os.path.join(project_root, "logs")
        os.makedirs(logs_dir, exist_ok=True)
        log_file = os.path.join(logs_dir, log_file)
    else:
        os.makedirs(os.path.dirname(log_file), exist_ok=True)

    # Create a file handler
    file_handler = WatchedFileHandler(log_file)
    file_handler.setLevel(level)
    file_handler.setFormatter(formatter)

    # Create a console handler (stdout)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)

    # Add handlers to the logger *only if* it doesn't have them
    if not logger.hasHandlers():
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
    
    # Propagate to root logger
    logger.propagate = False 

    return logger
