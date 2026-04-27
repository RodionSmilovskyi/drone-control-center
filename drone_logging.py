import logging
import sys
import os

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

    # Create a file handler
    file_handler = logging.FileHandler(log_file, mode='w') # 'w' to overwrite log on each run
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
