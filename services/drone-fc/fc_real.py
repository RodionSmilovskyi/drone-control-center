import time
import logging
from yamspy import MSPy

class FCReal:
    """Real provider for the flight controller using MSPy over a serial connection."""
    def __init__(self, serial_port="/dev/ttyACM0", baudrate=115200):
        self.logger = logging.getLogger("drone-fc-real")
        self.logger.info(f"Connecting to FC on {serial_port}...")
        
        self.board = MSPy(device=serial_port, loglevel='WARNING', baudrate=baudrate)
        if self.board == 1:
            raise Exception(f"Could not connect to flight controller at {serial_port}")

        self.logger.info("Connected! Starting initialization sequence...")
        
        # Essential initialization sequence to prevent failsafe
        command_list = [
            'MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES', 'MSP_ATTITUDE', 'MSP_ALTITUDE'
        ]
        
        for msg in command_list: 
            if self.board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = self.board.receive_msg()
                self.board.process_recv_data(dataHandler)
        
        self.logger.info("Initialization sequence complete.")

    def send_rc(self, rc_commands: list):
        """Sends RC commands to the flight controller."""
        # rc_commands: [roll, pitch, throttle, yaw, aux1, aux2]
        if self.board.send_RAW_RC(rc_commands):
            dataHandler = self.board.receive_msg()
            self.board.process_recv_data(dataHandler)
