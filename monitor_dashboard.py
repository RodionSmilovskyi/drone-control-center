import curses
import paho.mqtt.client as mqtt
import json
from datetime import datetime

# --- Configuration ---
MQTT_BROKER = "localhost"   # The Pi Zero is the broker
MQTT_PORT = 1883

# --- Topics to Monitor ---
SENSOR_TOPIC = "drone/sensors"
STATUS_TOPIC = "drone/status"
COMMAND_TOPIC = "drone/commands"

# This dictionary will hold the latest known state of the drone
latest_state = {
    SENSOR_TOPIC: {},
    STATUS_TOPIC: {},
    COMMAND_TOPIC: {},
    "last_update": {
        SENSOR_TOPIC: None,
        STATUS_TOPIC: None,
        COMMAND_TOPIC: None
    }
}

# We move on_connect inside draw_dashboard

def on_message(client, userdata, msg):
    """Callback for when a message is received on any subscribed topic."""
    global latest_state
    topic = msg.topic
    
    try:
        payload = json.loads(msg.payload.decode())
        
        # Update the state dictionary with the new payload
        if topic in latest_state:
            latest_state[topic] = payload
            latest_state["last_update"][topic] = datetime.now()
            
    except (json.JSONDecodeError, KeyError):
        pass # Ignore badly formatted messages

def draw_dashboard(stdscr):
    """This function is the main loop for the curses display."""

    # --- DEFINE on_connect HERE (MOVED) ---
    def on_connect(client, userdata, flags, rc, properties):
        """Callback for when the client connects to the broker."""
        # 'stdscr' is now available from the outer scope
        if rc == 0:
            # Subscribe to all topics we care about
            client.subscribe("drone/#")
            # We can safely draw to the screen
            stdscr.addstr(1, 2, "MQTT Connected.", curses.A_DIM)
        else:
            # Use stdscr to report the error
            stdscr.addstr(1, 2, f"MQTT Failed to connect, return code {rc}", curses.A_BOLD)

    # --- Curses Setup ---
    curses.curs_set(0) # Hide the cursor
    stdscr.nodelay(True) # Don't block for getch()
    stdscr.timeout(100) # Refresh up to 10 times per second (100ms)
    
    # --- MQTT Client Setup ---
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="drone_dashboard")
    client.on_connect = on_connect # Assign the inner function
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except ConnectionRefusedError:
        print("Connection to MQTT broker refused. Is it running?")
        return

    client.loop_start() # Start the non-blocking network loop

    # --- Main Display Loop ---
    while True:
        try:
            # Check for user input (to quit)
            char = stdscr.getch()
            if char == ord('q') or char == ord('Q'):
                break
                
            # --- Clear Screen ---
            stdscr.clear()
            
            # --- Draw Header ---
            stdscr.addstr(0, 2, "--- DRONE DASHBOARD (Press 'q' to quit) ---", curses.A_BOLD)
            
            # --- Draw Status ---
            stdscr.addstr(2, 2, "STATUS", curses.A_BOLD)
            status = latest_state[STATUS_TOPIC]
            armed = status.get("armed", False)
            armed_str = "ARMED" if armed else "DISARMED"
            armed_color = curses.color_pair(1) if armed else curses.color_pair(2)
            if curses.has_colors():
                curses.start_color()
                curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
                curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
                stdscr.addstr(3, 4, f"State:    {armed_str}", armed_color | curses.A_BOLD)
            else:
                stdscr.addstr(3, 4, f"State:    {armed_str}", curses.A_BOLD)

            # --- Draw Sensors ---
            stdscr.addstr(5, 2, "SENSORS", curses.A_BOLD)
            sensors = latest_state[SENSOR_TOPIC]
            alt = sensors.get("altitude", 0.0)
            kine = sensors.get("kinematics", [0, 0, 0])
            stdscr.addstr(6, 4, f"Altitude:   {alt:.2f} m")
            stdscr.addstr(7, 4, f"Kinematics: [R:{kine[0]:.1f}, P:{kine[1]:.1f}, Y:{kine[2]:.1f}]")
            
            # --- Draw Commands ---
            stdscr.addstr(9, 2, "LAST COMMAND", curses.A_BOLD)
            cmds = latest_state[COMMAND_TOPIC]
            stdscr.addstr(10, 4, f"Throttle: {cmds.get('throttle', 0)}")
            stdscr.addstr(11, 4, f"Roll:     {cmds.get('roll', 0)}")
            stdscr.addstr(12, 4, f"Pitch:    {cmds.get('pitch', 0)}")
            stdscr.addstr(13, 4, f"Yaw:      {cmds.get('yaw', 0)}")

            # --- Draw Footer (Last Update Time) ---
            stdscr.addstr(15, 2, "Last Sensor Update:  ", curses.A_DIM)
            if latest_state["last_update"][SENSOR_TOPIC]:
                stdscr.addstr(latest_state["last_update"][SENSOR_TOPIC].strftime('%H:%M:%S.%f')[:-3])

            stdscr.addstr(16, 2, "Last Command Update: ", curses.A_DIM)
            if latest_state["last_update"][COMMAND_TOPIC]:
                stdscr.addstr(latest_state["last_update"][COMMAND_TOPIC].strftime('%H:%M:%S.%f')[:-3])

            # --- Refresh Display ---
            stdscr.refresh()
            
        except KeyboardInterrupt:
            break

    # --- Cleanup ---
    client.loop_stop()

if __name__ == '__main__':
    # curses.wrapper handles all the terminal setup and cleanup
    curses.wrapper(draw_dashboard)

