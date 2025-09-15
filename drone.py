"""minimal_msp_controller.py: A minimalistic script to test YAMSPy communication.

This script connects to a flight controller, sends default RC commands,
and periodically requests and prints sensor data to the console. It also
handles keyboard input for basic flight control.

Based on the simpleUI.py example from the YAMSPy project.
"""

__author__ = "Ricardo de Azambuja"
__copyright__ = "Copyright 2020, MISTLab.ca"
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Ricardo de Azambuja"
__email__ = "ricardo.azambuja@gmail.com"
__status__ = "Development"

import time
import curses
from itertools import cycle

from yamspy import MSPy

# Loop timings
CTRL_LOOP_TIME = 1/100  # 100Hz loop for sending RC commands
SLOW_MSGS_LOOP_TIME = 1/5 # 5Hz loop for requesting other data

# Serial port configuration
SERIAL_PORT = "/dev/ttyACM0"

def run_curses(external_function):
    """Wrapper function to handle curses setup and teardown."""
    result = 1
    try:
        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        screen.timeout(0)
        screen.keypad(True)

        # Print instructions
        screen.addstr(0, 0, "Press 'q' to quit, 'a' to arm, 'd' to disarm.", curses.A_BOLD)
        screen.addstr(1, 0, "Use arrow keys for roll/pitch, 'w'/'e' for throttle.", curses.A_BOLD)
        
        result = external_function(screen)

    finally:
        # Shut down cleanly
        curses.nocbreak()
        screen.keypad(0)
        curses.echo()
        curses.endwin()
        if result == 1:
            print("An error occurred... probably the serial port is not available.")

def keyboard_controller(screen):
    """Main function to handle flight controller communication and keyboard input."""
    # Default command values
    CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 900,
        'yaw':      1500,
        'aux1':     1000, # Disarmed
        'aux2':     1000  # Angle Mode
    }

    # This order must match your flight controller's channel map (e.g., AETR)
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

    try:
        screen.addstr(3, 0, "Connecting to the FC...")
        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board == 1:
                return 1

            screen.addstr(3, 0, "Connecting to the FC... connected!")
            screen.clrtoeol()

            # Cycle through a list of slow messages to request periodically
            slow_msgs = cycle(['MSP_STATUS_EX', 'MSP_ATTITUDE', 'MSP_ALTITUDE', 'MSP_ANALOG'])
            
            last_loop_time = time.time()
            last_slow_msg_time = time.time()
            
            cursor_msg = "Starting main loop..."
            while True:
                current_time = time.time()

                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer

                # --- Key input processing ---
                if char == ord('q') or char == ord('Q'):
                    break
                elif char == ord('a') or char == ord('A'):
                    cursor_msg = "ARMing command sent."
                    CMDS['aux1'] = 1800
                elif char == ord('d') or char == ord('D'):
                    cursor_msg = "DISarm command sent."
                    CMDS['aux1'] = 1000
                elif char == ord('w') or char == ord('W'):
                    CMDS['throttle'] = min(2000, CMDS['throttle'] + 10)
                    cursor_msg = f"Throttle: {CMDS['throttle']}"
                elif char == ord('e') or char == ord('E'):
                    CMDS['throttle'] = max(900, CMDS['throttle'] - 10)
                    cursor_msg = f"Throttle: {CMDS['throttle']}"
                elif char == curses.KEY_RIGHT:
                    CMDS['roll'] = min(2000, CMDS['roll'] + 10)
                    cursor_msg = f"Roll: {CMDS['roll']}"
                elif char == curses.KEY_LEFT:
                    CMDS['roll'] = max(1000, CMDS['roll'] - 10)
                    cursor_msg = f"Roll: {CMDS['roll']}"
                elif char == curses.KEY_UP:
                    CMDS['pitch'] = min(2000, CMDS['pitch'] + 10)
                    cursor_msg = f"Pitch: {CMDS['pitch']}"
                elif char == curses.KEY_DOWN:
                    CMDS['pitch'] = max(1000, CMDS['pitch'] - 10)
                    cursor_msg = f"Pitch: {CMDS['pitch']}"


                # Fast loop for sending important commands (e.g., RC channels)
                if (current_time - last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = current_time
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[key] for key in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                # Slow loop for requesting less critical data for monitoring
                if (current_time - last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = current_time
                    next_msg = next(slow_msgs)

                    # Request and process the next message in the cycle
                    if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                        
                        screen.addstr(3, 0, f"Status: {cursor_msg}")
                        screen.clrtoeol()

                        if next_msg == 'MSP_ATTITUDE':
                            screen.addstr(5, 0, f"Attitude: {board.SENSOR_DATA['kinematics']}")
                            screen.clrtoeol()
                        
                        elif next_msg == 'MSP_ALTITUDE':
                            screen.addstr(6, 0, f"Altitude: {board.SENSOR_DATA['altitude']}")
                            screen.clrtoeol()

                        elif next_msg == 'MSP_STATUS_EX':
                            is_armed = board.bit_check(board.CONFIG['mode'], 0)
                            screen.addstr(7, 0, f"ARMED: {is_armed}, Flight Mode: {board.process_mode(board.CONFIG['mode'])}")
                            screen.clrtoeol()
                        
                        elif next_msg == 'MSP_ANALOG':
                            screen.addstr(8, 0, f"Voltage: {board.ANALOG.get('voltage', 'N/A')}V")
                            screen.clrtoeol()
                
                # Small delay to prevent busy-waiting and reduce CPU usage
                time.sleep(CTRL_LOOP_TIME - (time.time() - current_time)) if (time.time() - current_time) < CTRL_LOOP_TIME else time.sleep(0.001)

    except Exception as e:
        # In curses, exceptions don't print well. Write to a file for debugging.
        with open("error.log", "w") as f:
            f.write(f"An error occurred: {e}")
        return 1

if __name__ == "__main__":
    run_curses(keyboard_controller)

