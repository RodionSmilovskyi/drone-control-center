import logging

class FCMock:
    """Mock provider for the flight controller that logs RC commands instead of sending them."""
    def __init__(self):
        logging.info("FCMock initialized.")

    def send_rc(self, rc_commands: list):
        """Mock sending RC commands."""
        # Optional: Add debug logging if needed, but keep it quiet for 100Hz loop
        pass
