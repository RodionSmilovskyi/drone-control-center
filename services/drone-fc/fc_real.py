import time
import logging
import glob
import os
import struct
from yamspy import MSPy

MSP2_SENSOR_RANGEFINDER = 0x1F01
MSP2_SENSOR_OPTICALFLOW = 0x1F02

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

    def send_sensors(self, altitude_m: float, raw_dx: float, raw_dy: float):
        """
        Forwards rangefinder altitude and optical flow pixel deltas to iNAV via MSPv2.
        IMPORTANT: Do NOT call receive_msg() after send_RAW_msg() for 0x1F01/0x1F02
        because iNAV returns MSP_RESULT_NO_REPLY for sensor injection frames.
        """
        if not self.board:
            return
        try:
            # 1. Rangefinder (0x1F01): <Bi (uint8 quality, int32 distance_mm)
            if altitude_m > 0.005:
                dist_mm = int(round(altitude_m * 1000.0))
                rf_payload = struct.pack("<Bi", 255, dist_mm)
            else:
                rf_payload = struct.pack("<Bi", 0, -1)
            self.board.send_RAW_msg(MSP2_SENSOR_RANGEFINDER, data=list(rf_payload))

            # 2. Optical Flow (0x1F02): <Bii (uint8 quality, int32 motionX, int32 motionY)
            of_payload = struct.pack("<Bii", 255, int(round(raw_dx)), int(round(raw_dy)))
            self.board.send_RAW_msg(MSP2_SENSOR_OPTICALFLOW, data=list(of_payload))
        except Exception as e:
            self.logger.error(f"send_sensors error: {e}")

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
