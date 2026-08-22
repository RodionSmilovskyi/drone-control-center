import time
import logging
import sys
import select
import termios
import tty
import os
import fcntl

from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich import box

from yamspy import MSPy

# Logging setup
LOG_FILE = "rich_ui.log"
logger = logging.getLogger('rich_ui')
logger.setLevel(logging.INFO)
logger.propagate = False
log_handler = logging.FileHandler(LOG_FILE, mode='w')
log_handler.setFormatter(logging.Formatter('%(asctime)s  %(message)s', datefmt='%H:%M:%S'))
logger.addHandler(log_handler)

logging.getLogger('MSPy').setLevel(logging.WARNING)

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
CTRL_LOOP_TIME = 0.01   # 100Hz RC updates
SLOW_LOOP_TIME = 0.2    # 5Hz Telemetry updates

class KeyboardPoller:
    """Non-blocking keyboard input on Linux."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        self.old_flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        
        tty.setcbreak(self.fd)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags)

    def get_char(self):
        try:
            return sys.stdin.read(1)
        except (IOError, TypeError):
            return None

class DroneController:
    def __init__(self, port, baudrate):
        self.board = MSPy(device=port, loglevel='WARNING', baudrate=baudrate)
        self.connected = False
        
        # AETR1234 mapping
        # A, E, T, R, AUX1, AUX2, AUX3, AUX4
        self.channels = {
            'roll': 1500,     # fixed neutral
            'pitch': 1500,    # fixed neutral
            'throttle': 900,  # dynamic
            'yaw': 1500,      # fixed neutral
            'aux1': 1000,     # arm/disarm
            'aux2': 1000,
            'aux3': 1000,
            'aux4': 1000
        }
        self.channel_order = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']
        
        # Telemetry State
        self.state = {
            "msg": "Connecting...",
            "fc_version": "N/A",
            "battery": "N/A",
            "armed": False,
            "mode": "N/A",
            "cpuload": 0,
            "altitude": 0.0,
            "hz": 0
        }

    def connect(self):
        if self.board == 1:
            return False
        self.connected = True
        self.state["msg"] = "Connected. Initializing..."
        
        # Initial requests
        init_cmds = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                     'MSP_BOARD_INFO', 'MSP_NAME', 'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE']
        for cmd in init_cmds:
            if self.board.send_RAW_msg(MSPy.MSPCodes[cmd], data=[]):
                self.board.process_recv_data(self.board.receive_msg())

        self.state["fc_version"] = f"{self.board.CONFIG['flightControllerIdentifier']} {self.board.CONFIG['flightControllerVersion']}"
        self.state["msg"] = "Ready. Press 'a' (Arm), 'd' (Disarm), 'w' (Throttle+), 's' (Throttle-), 'q' (Quit)"
        return True

    def process_input(self, char):
        if not char:
            return True
            
        char = char.lower()
        if char == 'q':
            return False
        elif char == 'a':
            self.channels['aux1'] = 1800
            self.state["msg"] = "[red]ARMING COMMAND SENT[/red]"
            logger.info("Armed")
        elif char == 'd':
            self.channels['aux1'] = 1000
            self.state["msg"] = "[green]DISARM COMMAND SENT[/green]"
            logger.info("Disarmed")
        elif char == 'w':
            self.channels['throttle'] = min(2000, self.channels['throttle'] + 25)
            self.state["msg"] = f"Throttle increased: {self.channels['throttle']}"
        elif char == 's':
            self.channels['throttle'] = max(900, self.channels['throttle'] - 25)
            self.state["msg"] = f"Throttle decreased: {self.channels['throttle']}"
            
        return True

    def send_rc(self):
        rc_data = [self.channels[ki] for ki in self.channel_order]
        if self.board.send_RAW_RC(rc_data):
            self.board.process_recv_data(self.board.receive_msg())

    def update_telemetry(self):
        cmds = ['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_ALTITUDE']
        for cmd in cmds:
            if self.board.send_RAW_msg(MSPy.MSPCodes[cmd], data=[]):
                self.board.process_recv_data(self.board.receive_msg())
        
        voltage = self.board.ANALOG.get('voltage', 0)
        self.state["battery"] = f"{voltage:.2f}V"
        
        self.state["armed"] = bool(self.board.bit_check(self.board.CONFIG.get('mode', 0), 0))
        self.state["cpuload"] = self.board.CONFIG.get('cpuload', 0)
        self.state["altitude"] = self.board.SENSOR_DATA.get('altitude', 0)
        self.state["mode"] = str(self.board.process_mode(self.board.CONFIG.get('mode', 0)))

    def build_ui(self) -> Layout:
        layout = Layout()
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main"),
            Layout(name="footer", size=3)
        )
        
        # Header
        header = Panel(Text("Drone Control Dashboard", justify="center", style="bold white on blue"), box=box.ROUNDED)
        layout["header"].update(header)
        
        # Main Dashboard
        grid = Table.grid(padding=3)
        grid.add_column("Key", style="cyan", justify="right")
        grid.add_column("Value", style="magenta")
        grid.add_column("Key2", style="cyan", justify="right")
        grid.add_column("Value2", style="magenta")
        
        armed_style = "bold red blink" if self.state["armed"] else "bold green"
        armed_text = "ARMED" if self.state["armed"] else "DISARMED"
        
        grid.add_row("FC Version:", self.state["fc_version"], "Status:", f"[{armed_style}]{armed_text}[/{armed_style}]")
        grid.add_row("Battery:", self.state["battery"], "Flight Mode:", self.state["mode"])
        grid.add_row("CPU Load:", f"{self.state['cpuload']}%", "Loop Hz:", f"{self.state['hz']:.1f}")
        grid.add_row("Altitude:", f"{self.state['altitude']}cm")
        
        grid.add_row()
        grid.add_row("[yellow]Throttle:[/yellow]", f"[bold white]{self.channels['throttle']}[/bold white]")
        
        layout["main"].update(Panel(grid, title="System Telemetry & Controls", box=box.ROUNDED, border_style="blue"))
        
        # Footer
        footer = Panel(Text(self.state["msg"], style="bold green", justify="center"), box=box.ROUNDED)
        layout["footer"].update(footer)
        
        return layout

def main():
    logger.info("Starting Betaflight Drone Controller")
    
    drone = DroneController(SERIAL_PORT, BAUD_RATE)
    
    with drone.board:
        if not drone.connect():
            print("ERROR: Could not connect to Flight Controller.")
            return

        last_rc_time = time.time()
        last_tel_time = time.time()
        
        loop_times = []
        
        with KeyboardPoller() as poller, Live(drone.build_ui(), refresh_per_second=20) as live:
            while True:
                start_time = time.time()
                
                # 1. Input processing
                char = poller.get_char()
                if not drone.process_input(char):
                    break
                    
                # 2. Fast RC Loop (100Hz)
                if (time.time() - last_rc_time) >= CTRL_LOOP_TIME:
                    drone.send_rc()
                    last_rc_time = time.time()
                    
                # 3. Slow Telemetry Loop (5Hz)
                if (time.time() - last_tel_time) >= SLOW_LOOP_TIME:
                    drone.update_telemetry()
                    last_tel_time = time.time()
                    live.update(drone.build_ui())
                    
                # Calculate Hz
                elapsed = time.time() - start_time
                loop_times.append(elapsed)
                if len(loop_times) > 20:
                    loop_times.pop(0)
                avg_loop = sum(loop_times) / len(loop_times)
                drone.state["hz"] = 1.0 / avg_loop if avg_loop > 0 else 0
                
                # Sleep if we're running too fast
                sleep_time = CTRL_LOOP_TIME - (time.time() - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

if __name__ == "__main__":
    main()
