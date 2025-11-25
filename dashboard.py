import curses
import paho.mqtt.client as mqtt
import json
from datetime import datetime

# --- Configuration ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# --- Topics to Monitor ---
SENSOR_TOPIC = "drone/sensors"
STATUS_TOPIC = "drone/status"
COMMAND_TOPIC = "drone/commands"
AI_MODE_TOPIC = "drone/ai_mode"

# This dictionary will hold the latest known state of the drone
latest_state = {
    SENSOR_TOPIC: {},
    STATUS_TOPIC: {},
    COMMAND_TOPIC: {},
    AI_MODE_TOPIC: {"ai_enabled": False},
    "last_update": {
        SENSOR_TOPIC: None,
        STATUS_TOPIC: None,
        COMMAND_TOPIC: None,
        AI_MODE_TOPIC: None 
    }
}

def on_message(client, userdata, msg):
    """Callback for when a message is received on any subscribed topic."""
    global latest_state
    topic = msg.topic
    
    try:
        payload = json.loads(msg.payload.decode())
        
        # Update the state dictionary with the new payload
        if topic in latest_state:
            # Merge dictionaries for sensor topic so we don't lose data 
            # if sensors report asynchronously (e.g. flow vs altitude)
            if topic == SENSOR_TOPIC:
                latest_state[topic].update(payload)
            else:
                latest_state[topic] = payload
                
            latest_state["last_update"][topic] = datetime.now()
            
    except (json.JSONDecodeError, KeyError):
        pass # Ignore badly formatted messages

def draw_dashboard(stdscr):
    """This function is the main loop for the curses display."""

    # --- DEFINE on_connect HERE ---
    def on_connect(client, userdata, flags, reason_code, properties):
        """Callback for when the client connects to the broker."""
        if reason_code == 0:
            client.subscribe("drone/#")
            try:
                stdscr.addstr(1, 2, "MQTT Connected.", curses.A_DIM)
            except curses.error:
                pass 
        else:
            try:
                stdscr.addstr(1, 2, f"MQTT Failed to connect, return code {reason_code}", curses.A_BOLD)
            except curses.error:
                pass

    # --- Curses Setup ---
    curses.curs_set(0) 
    stdscr.nodelay(True) 
    stdscr.timeout(100) 
    
    # --- Colors ---
    if curses.has_colors():
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK) # ARMED
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)   # DISARMED
        curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)  # AI ENABLED
    
    # --- MQTT Client Setup ---
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="drone_dashboard")
    client.on_connect = on_connect 
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except ConnectionRefusedError:
        print("Connection to MQTT broker refused. Is it running?")
        return

    client.loop_start() 

    # --- Main Display Loop ---
    while True:
        try:
            char = stdscr.getch()
            if char == ord('q') or char == ord('Q'):
                break
                
            stdscr.clear()
            
            # --- Draw Header ---
            stdscr.addstr(0, 2, "--- DRONE DASHBOARD (Press 'q' to quit) ---", curses.A_BOLD)
            
            # --- Draw Status ---
            stdscr.addstr(2, 2, "STATUS", curses.A_BOLD)
            status = latest_state[STATUS_TOPIC]
            armed = status.get("armed", False)
            armed_str = "ARMED" if armed else "DISARMED"
            armed_color = curses.color_pair(1) if armed else curses.color_pair(2)
            
            stdscr.addstr(3, 4, f"FC State: ")
            stdscr.addstr(f"{armed_str}", armed_color | curses.A_BOLD)

            # --- Draw AI Mode ---
            ai_status = latest_state[AI_MODE_TOPIC]
            ai_enabled = ai_status.get("ai_enabled", False)
            ai_str = "ENABLED" if ai_enabled else "DISABLED"
            ai_color = curses.color_pair(3) if ai_enabled else curses.A_DIM
            
            stdscr.addstr(4, 4, f"AI Mode:  ")
            stdscr.addstr(f"{ai_str}", ai_color | curses.A_BOLD)


            # --- Draw Sensors ---
            stdscr.addstr(6, 2, "SENSORS", curses.A_BOLD)
            sensors = latest_state[SENSOR_TOPIC]
            
            # Altitude
            alt = sensors.get("altitude", 0.0)
            stdscr.addstr(7, 4, f"Altitude:   {alt:.2f} m")
            
            # Kinematics
            kine = sensors.get("kinematics", [0, 0, 0])
            stdscr.addstr(8, 4, f"Kinematics: [R:{kine[0]:.1f}, P:{kine[1]:.1f}, Y:{kine[2]:.1f}]")
            
            # Obstacle Distance (NEW)
            obs = sensors.get("obstacle_distance", 0.0)
            stdscr.addstr(9, 4, f"Obstacle:   {obs:.2f} m")
            
            # Optical Flow (NEW)
            flow = sensors.get("flow", {'x': 0, 'y': 0})
            # Handle case where flow might be None if sensor timed out
            if flow is None: flow = {'x': 0, 'y': 0} 
            stdscr.addstr(10, 4, f"Flow:       X:{flow.get('x', 0):>3}  Y:{flow.get('y', 0):>3}")
            
            # --- Draw Commands ---
            stdscr.addstr(12, 2, "LAST COMMAND", curses.A_BOLD)
            cmds = latest_state[COMMAND_TOPIC]
            stdscr.addstr(13, 4, f"Throttle: {cmds.get('throttle', 0)}")
            stdscr.addstr(14, 4, f"Roll:     {cmds.get('roll', 0)}")
            stdscr.addstr(15, 4, f"Pitch:    {cmds.get('pitch', 0)}")
            stdscr.addstr(16, 4, f"Yaw:      {cmds.get('yaw', 0)}")

            # --- Draw Footer (Last Update Time) ---
            stdscr.addstr(18, 2, "Last Sensor Update:  ", curses.A_DIM)
            if latest_state["last_update"][SENSOR_TOPIC]:
                stdscr.addstr(latest_state["last_update"][SENSOR_TOPIC].strftime('%H:%M:%S.%f')[:-3])

            stdscr.addstr(19, 2, "Last Command Update: ", curses.A_DIM)
            if latest_state["last_update"][COMMAND_TOPIC]:
                stdscr.addstr(latest_state["last_update"][COMMAND_TOPIC].strftime('%H:%M:%S.%f')[:-3])

            stdscr.refresh()
            
        except KeyboardInterrupt:
            break
        except curses.error:
            pass

    # --- Cleanup ---
    client.loop_stop()
    client.disconnect()

if __name__ == '__main__':
    curses.wrapper(draw_dashboard)