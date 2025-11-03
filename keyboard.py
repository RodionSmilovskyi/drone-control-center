import logging
import time
import json
import curses
import paho.mqtt.client as mqtt
from drone_logging import setup_logger

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
LOG_FILE = "keyboard_teleop.log"

# --- MQTT Topics ---
COMMAND_TOPIC = "drone/commands"
SYSTEM_TOPIC = "drone/system_command"
AI_MODE_TOPIC = "drone/ai_mode" # NEW: Topic to publish AI state

# --- Setup Logger ---
logger = setup_logger("Keyboard_Teleop", LOG_FILE)

# --- Main Logic ---
def main(stdscr):
    # --- Curses Setup ---
    curses.curs_set(0) # Hide cursor
    stdscr.nodelay(True) # Don't block
    stdscr.timeout(100) # Refresh 10 times/sec
    
    # --- MQTT Client ---
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="keyboard_teleop")
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    # --- State Variables ---
    # Start with a safe, disarmed state
    rc_commands = {'roll': 1500, 'pitch': 1500, 'throttle': 900, 'yaw': 1500, 'aux1': 1000, 'aux2': 1000}
    ai_mode_enabled = False # NEW: AI is disabled by default
    
    logger.info("Keyboard Teleop running. Press 'q' to quit.")

    while True:
        try:
            char = stdscr.getch()
            curses.flushinp()

            # --- Key Handling ---
            if char == ord('q') or char == ord('Q'):
                logger.info("Quit command received. Exiting.")
                break
            
            # --- AI Toggle ---
            elif char == ord('x') or char == ord('X'):
                ai_mode_enabled = not ai_mode_enabled
                logger.info(f"AI Mode Toggled: {'ENABLED' if ai_mode_enabled else 'DISABLED'}")
            
            # --- System Commands ---
            elif char == ord('c') or char == ord('C'):
                logger.info("Sending CALIBRATE command.")
                client.publish(SYSTEM_TOPIC, json.dumps({"command": "calibrate"}))
            
            # --- RC Commands (Arm/Disarm) ---
            elif char == ord('a') or char == ord('A'):
                rc_commands['aux1'] = 1800 # ARM
                logger.info("Sending ARM command (AUX1: 1800).")
            elif char == ord('d') or char == ord('D'):
                rc_commands['aux1'] = 1000 # DISARM
                ai_mode_enabled = False # SAFETY: Disarm also disables AI
                logger.info("Sending DISARM command (AUX1: 1000). AI DISABLED.")

            # --- RC Commands (Throttle) ---
            elif char == ord('w') or char == ord('W'):
                rc_commands['throttle'] = min(2000, rc_commands['throttle'] + 25)
                logger.debug(f"Throttle Inc: {rc_commands['throttle']}")
            elif char == ord('s') or char == ord('S'):
                rc_commands['throttle'] = max(900, rc_commands['throttle'] - 25)
                logger.debug(f"Throttle Dec: {rc_commands['throttle']}")

            # --- RC Commands (Roll/Pitch) ---
            elif char == curses.KEY_UP:
                rc_commands['pitch'] = 1600 # Simple forward
            elif char == curses.KEY_DOWN:
                rc_commands['pitch'] = 1400 # Simple backward
            elif char == curses.KEY_LEFT:
                rc_commands['roll'] = 1400 # Simple left
            elif char == curses.KEY_RIGHT:
                rc_commands['roll'] = 1600 # Simple right
            
            # --- Reset sticks if no key is pressed ---
            elif char == -1: # No key pressed
                rc_commands['roll'] = 1500
                rc_commands['pitch'] = 1500
                rc_commands['yaw'] = 1500
            
            # --- Publish Commands ---
            # Publish RC commands *unless* AI is active.
            # When AI is active, the tactical_controller is in charge.
            if not ai_mode_enabled:
                client.publish(COMMAND_TOPIC, json.dumps(rc_commands))
                logger.debug(f"Pub (Manual): {rc_commands}")
            
            # ALWAYS publish the AI mode state
            client.publish(AI_MODE_TOPIC, json.dumps({"ai_enabled": ai_mode_enabled}))
            
            # --- Update Display ---
            stdscr.clear()
            stdscr.addstr(0, 2, "--- MANUAL TELEOPERATION (Press 'q' to quit) ---", curses.A_BOLD)
            stdscr.addstr(2, 4, f"Throttle: {rc_commands['throttle']}   Roll: {rc_commands['roll']}   Pitch: {rc_commands['pitch']}")
            stdscr.addstr(3, 4, f"Arm (AUX1): {rc_commands['aux1']}")
            
            stdscr.addstr(5, 4, "[a] Arm   [d] Disarm   [c] Calibrate   [w/s] Throttle   [arrows] Roll/Pitch")
            
            stdscr.addstr(7, 4, f"AI MODE: ", curses.A_BOLD)
            if ai_mode_enabled:
                stdscr.addstr("ENABLED (Press 'x' to disable)", curses.A_REVERSE)
            else:
                stdscr.addstr("DISABLED (Press 'x' to enable)")

            stdscr.refresh()

        except KeyboardInterrupt:
            logger.info("KeyboardTeleop stopping...")
            break
        except Exception as e:
            logger.error(f"Error in Teleop loop: {e}")
            break
            
    client.loop_stop()
    client.disconnect()

if __name__ == '__main__':
    logger.setLevel(logging.INFO) # or logging.DEBUG
    curses.wrapper(main)

