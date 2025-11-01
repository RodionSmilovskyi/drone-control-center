import json
import logging
import paho.mqtt.client as mqtt
import curses
import time
from drone_logging import setup_logger

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
COMMAND_TOPIC = "drone/commands"
STATUS_TOPIC = "drone/status"
SYSTEM_TOPIC = "drone/system_command" # New topic for commands like 'calibrate'
LOG_FILE = "keyboard.log"

logger = setup_logger("Teleop", LOG_FILE)

# --- Main Logic ---

def keyboard_controller(screen):
    """Main Curses loop for simplified keyboard control."""
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="keyboard_teleop")
    
    # Updated V2 API on_connect signature
    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            screen.addstr(2, 0, "Teleop Connected to MQTT Broker!", curses.A_DIM)
            client.subscribe(STATUS_TOPIC)
        else:
            screen.addstr(2, 0, f"Failed to connect: {reason_code}")
        screen.refresh()

    def on_status_message(client, userdata, msg):
        try:
            status = json.loads(msg.payload.decode())
            is_armed = status.get("armed", False)
            armed_str = "ARMED" if is_armed else "DISARMED"
            screen.addstr(5, 0, f"ARMED: {armed_str}", curses.A_BOLD)
            screen.clrtoeol()
        except json.JSONDecodeError:
            pass

    client.on_connect = on_connect
    # Register the status message callback
    client.message_callback_add(STATUS_TOPIC, on_status_message)

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except ConnectionRefusedError:
        screen.addstr(2, 0, "Could not connect to MQTT Broker. Is it running?")
        screen.refresh()
        logger.error("Teleop: Connection to MQTT broker refused. Is it running?")
        time.sleep(3)
        return

    client.loop_start()

    # Initial command state (simplified)
    cmds = {
        'roll': 1500, 'pitch': 1500, 'throttle': 900, 'yaw': 1500,
        'aux1': 1000, 'aux2': 1000  # Disarmed
    }

    cursor_msg = "Awaiting commands..."
    
    while True:
        screen.addstr(3, 0, cursor_msg)
        screen.clrtoeol()
        screen.addstr(4, 0, f"Throttle: {cmds['throttle']} | Arm (aux1): {cmds['aux1']}")
        screen.clrtoeol()
        
        char = screen.getch()  # Get keypress
        curses.flushinp()  # Flushes buffer

        # --- Simplified Key input processing ---
        if char == ord('q') or char == ord('Q'):
            logger.info("Quit command received.")
            break
        elif char == ord('a') or char == ord('A'):
            cursor_msg = 'Sending Arm command (aux1 = 1800)'
            logger.info("Sending ARM command.")
            cmds['aux1'] = 1800
        elif char == ord('d') or char == ord('D'):
            cursor_msg = 'Sending Disarm command (aux1 = 1000)'
            logger.info("Sending DISARM command.")
            cmds['aux1'] = 1000
        # Throttle
        elif char == ord('w') or char == ord('W'):
            cmds['throttle'] = min(2000, cmds['throttle'] + 10)
            cursor_msg = f"Throttle (+): {cmds['throttle']}"
            logger.info(f"Throttle UP: {cmds['throttle']}")
            
        elif char == ord('s') or char == ord('S'):
            cmds['throttle'] = max(900, cmds['throttle'] - 10)
            cursor_msg = f"Throttle (-): {cmds['throttle']}"
            logger.info(f"Throttle DOWN: {cmds['throttle']}")
        # Calibrate
        elif char == ord('c') or char == ord('C'):
            cursor_msg = 'Sending CALIBRATE command...'
            # Publish to the new system topic
            client.publish(SYSTEM_TOPIC, json.dumps({"command": "calibrate"}))
            # We don't publish to the main command topic, as this isn't an RC 
            logger.info("Sending CALIBRATE command.")
            continue 

        # Publish the current RC command state
        client.publish(COMMAND_TOPIC, json.dumps(cmds))
        
        time.sleep(0.02) # Send commands at ~50Hz

    client.loop_stop()

def run_curses(external_function):
    """Wrapper to handle Curses setup and teardown."""
    try:
        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        screen.timeout(20) # Set a small timeout
        screen.keypad(True)
        screen.addstr(1, 0, "Press 'q' to quit, 'a' to arm, 'd' to disarm, 'c' to calibrate, 'w'/'s' for throttle", curses.A_BOLD)
        external_function(screen)
    finally:
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()

if __name__ == "__main__":
    logger.setLevel(logging.INFO) # or logging.DEBUG
    run_curses(keyboard_controller)

