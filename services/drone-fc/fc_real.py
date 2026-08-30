import time
import logging
import glob
import os
from yamspy import MSPy

class FCReal:
    """Real provider for the flight controller using MSPy over a serial connection."""
    def __init__(self, serial_port=None, baudrate=115200, max_retries=5):
        self.logger = logging.getLogger("drone-fc-real")
        self.baudrate = baudrate
        self.board = None
        self.active_port = None
        
        # Determine candidate serial ports (auto-detect /dev/ttyACM* and /dev/ttyUSB*)
        if serial_port:
            candidate_ports = [serial_port]
        else:
            detected = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            candidate_ports = detected if detected else ["/dev/ttyACM0", "/dev/ttyACM1"]
        
        last_error = None
        for attempt in range(1, max_retries + 1):
            for port in candidate_ports:
                if not os.path.exists(port) and len(candidate_ports) > 1:
                    continue
                try:
                    self.logger.info(f"[Attempt {attempt}/{max_retries}] Connecting to FC on {port}...")
                    board = MSPy(device=port, loglevel='WARNING', baudrate=self.baudrate)
                    if board.__enter__() == 1:
                        try:
                            board.__exit__(None, None, None)
                        except Exception:
                            pass
                        raise Exception(f"Could not connect to flight controller at {port}")
                    
                    self.board = board
                    self.active_port = port
                    self.logger.info(f"Connected to FC on {port}! Running initialization sequence...")
                    self._init_sequence()
                    self.logger.info(f"FC initialization sequence complete on {port}.")
                    return
                except Exception as e:
                    last_error = e
                    self.logger.warning(f"Connection attempt on {port} failed: {e}")
                    if self.board:
                        try:
                            self.board.__exit__(None, None, None)
                        except Exception:
                            pass
                        self.board = None
                    time.sleep(0.5)
            
            time.sleep(0.5)
            # Re-scan ports in case USB re-enumerated during sleep
            detected = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if detected:
                candidate_ports = detected
                
        raise Exception(f"Failed to connect to flight controller after {max_retries} attempts. Last error: {last_error}")

    def _init_sequence(self):
        # Essential initialization sequence to prevent failsafe
        command_list = [
            'MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES', 'MSP_ATTITUDE', 'MSP_ALTITUDE'
        ]

        if hasattr(self.board, 'INAV') and self.board.INAV:
            command_list.append('MSPV2_INAV_ANALOG')
            command_list.append('MSP_VOLTAGE_METER_CONFIG')
        
        for msg in command_list: 
            try:
                if self.board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = self.board.receive_msg()
                    self.board.process_recv_data(dataHandler)
            except Exception as e:
                self.logger.debug(f"Init msg {msg} skipped: {e}")

    def send_rc(self, rc_commands: list):
        """Sends RC commands to the flight controller."""
        # rc_commands: [roll, pitch, throttle, yaw, aux1, aux2]
        if self.board:
            try:
                if self.board.send_RAW_RC(rc_commands):
                    dataHandler = self.board.receive_msg()
                    self.board.process_recv_data(dataHandler)
            except Exception as e:
                self.logger.error(f"send_rc error: {e}")

    def close(self):
        """Closes the connection to the flight controller with safe disarm."""
        if hasattr(self, 'board') and self.board:
            try:
                # Send safe disarm pulse before closing
                self.board.send_RAW_RC([1500, 1500, 900, 1500, 1000, 1000])
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self.board.__exit__(None, None, None)
            except Exception as e:
                self.logger.error(f"Error closing FC board: {e}")
            self.board = None
