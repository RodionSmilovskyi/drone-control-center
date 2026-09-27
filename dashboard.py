import time
import os
import numpy as np
import subprocess
import signal
import sys
import select
import tty
import termios
import threading
import zmq
import json
from rich.console import Console
from rich.layout import Layout
from rich.panel import Panel
from rich.live import Live
from rich.text import Text
from rich.table import Table
from core.shared_memory_manager import SharedMemoryManager

# Global state
shutting_down = False
current_mode = "disarmed"
target_altitude = 0.4
mode_lock = threading.Lock()

def publish_mode_cmd(pub_socket):
    """Sends current flight mode and target altitude via non-blocking ZMQ."""
    payload = json.dumps({"mode": current_mode, "target_alt": round(target_altitude, 2)})
    pub_socket.send_string(payload)

def signal_handler(sig, frame):
    global shutting_down
    shutting_down = True

def get_status_table(heartbeats: np.ndarray) -> Table:
    """Creates the status block with dynamic health for all core services."""
    table = Table.grid(padding=(0, 1))
    table.add_column(style="bold white")
    table.add_column()

    def get_status_style(hb_time):
        if hb_time > 0 and (time.time() - hb_time) < 1.0:
            return "Ok", "bold green"
        return "Fail", "bold red"

    # Indices: 0: Sensors, 1: Inference, 2: FC
    service_map = [
        ("Sensors:", heartbeats[0]),
        ("Inference:", heartbeats[1]),
        ("FC:", heartbeats[2]),
    ]

    for label, hb in service_map:
        text, style = get_status_style(hb)
        table.add_row(label, f"[{style}]{text}[/{style}]")
    
    return table

def get_sensor_table(data: np.ndarray) -> Table:
    """Creates a table for sensor readings."""
    table = Table(title="Live Sensor Data", show_header=True, header_style="bold magenta")
    table.add_column("Sensor", style="dim")
    table.add_column("Value", justify="right")
    
    labels = ["Altitude", "Front Dist", "Shift X", "Shift Y", "Velocity X", "Velocity Y"]
    for i, label in enumerate(labels):
        val = data[i] if i < len(data) else 0.0
        table.add_row(label, f"{val: .3f}")
    
    # Show heartbeat age
    heartbeat = data[6] if len(data) > 6 else 0.0
    diff = time.time() - heartbeat if heartbeat > 0 else 0.0
    table.add_section()
    table.add_row("Heartbeat age", f"[cyan]{diff: .2f}s[/cyan]")
    
    return table

def get_rc_table(rc_commands: list) -> Table:
    """Creates a table for RC commands."""
    table = Table(title="RC Commands", show_header=True, header_style="bold yellow")
    table.add_column("Channel", style="dim")
    table.add_column("Value", justify="right")
    
    channels = ["Roll", "Pitch", "Throttle", "Yaw", "Aux1", "Aux2"]
    for i, label in enumerate(channels):
        val = rc_commands[i] if i < len(rc_commands) else 1000
        table.add_row(label, str(val))
    
    return table

def get_mode_panel(mode: str, rc_commands: list, target_alt: float) -> Panel:
    """Creates a panel displaying the current operating mode, target altitude, and RC summary."""
    mode_colors = {
        "disarmed": "bold red",
        "armed": "bold green",
        "ai": "bold yellow"
    }
    color = mode_colors.get(mode, "bold white")
    
    mode_text = Text(f"{mode.upper()}", style=f"{color} underline")
    target_text = Text(f"\nTarget Alt: {target_alt:.2f}m", style="bold cyan")
    rc_summary = Text(f"\nRC: {rc_commands[:4]}", style="dim")
    
    content = Text.assemble("\n", mode_text, "\n", target_text, "\n", rc_summary)
    
    return Panel(content, title="[bold]System Status[/bold]", border_style=color.split()[-1], title_align="center")

def generate_dashboard(sensor_data: np.ndarray, heartbeats: np.ndarray, mode: str, rc_commands: list, target_alt: float = 0.4, is_exiting: bool = False) -> Panel:
    """Generates the full dashboard panel."""
    if is_exiting:
        return Panel(
            Text("\n\nSYSTEM SHUTTING DOWN...\n\n", style="bold red", justify="center"),
            title="[bold blue]Drone Control Center[/bold blue]",
            border_style="red",
            expand=True
        )

    status_table = get_status_table(heartbeats)
    sensor_table = get_sensor_table(sensor_data)
    rc_table = get_rc_table(rc_commands)
    mode_panel = get_mode_panel(mode, rc_commands, target_alt)
    
    system_ok = (heartbeats[0] > 0 and (time.time() - heartbeats[0]) < 1.0) and \
                (heartbeats[1] > 0 and (time.time() - heartbeats[1]) < 1.0) and \
                (heartbeats[2] > 0 and (time.time() - heartbeats[2]) < 1.0)
    
    status_block = Panel(
        status_table,
        title="[bold]Statuses[/bold]",
        border_style="green" if system_ok else "red",
        expand=False
    )
    
    # Create a layout for the internal content
    inner_layout = Layout()
    inner_layout.split_row(
        Layout(status_block, size=35),
        Layout(mode_panel, name="center"),
        Layout(Panel(rc_table, border_style="yellow"), name="rc"),
        Layout(Panel(sensor_table, border_style="cyan"), name="right")
    )
    
    return Panel(
        inner_layout,
        title="[bold blue]Drone Control Center[/bold blue]",
        border_style="bright_blue",
        expand=True
    )

def keyboard_listener(pub_socket):
    """Background thread to handle keyboard input."""
    global current_mode, target_altitude, shutting_down
    while not shutting_down:
        try:
            # sys.stdin.read(1) will block in this thread, which is fine
            char = sys.stdin.read(1)
            if not char:
                break
            
            new_mode = None
            char_lower = char.lower()
            if char_lower == 'a':
                new_mode = "armed"
            elif char_lower == 'd':
                new_mode = "disarmed"
            elif char_lower == 'x':
                new_mode = "ai"
            elif char_lower == 'w':
                with mode_lock:
                    target_altitude = min(2.5, round(target_altitude + 0.05, 2))
                    publish_mode_cmd(pub_socket)
            elif char_lower == 's':
                with mode_lock:
                    target_altitude = max(0.1, round(target_altitude - 0.05, 2))
                    publish_mode_cmd(pub_socket)
            elif char_lower == 'q':
                shutting_down = True
                break
            
            if new_mode:
                with mode_lock:
                    if current_mode != new_mode:
                        current_mode = new_mode
                        publish_mode_cmd(pub_socket)
        except Exception:
            break

def main():
    console = Console()
    
    # Clear all .log files in logs/ so logging starts fresh
    try:
        logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        if os.path.exists(logs_dir):
            for filename in os.listdir(logs_dir):
                if filename.endswith(".log"):
                    file_path = os.path.join(logs_dir, filename)
                    try:
                        # Truncate file so active file handles write from scratch
                        with open(file_path, "w") as f:
                            pass
                    except Exception as fe:
                        console.print(f"[yellow]Warning: Could not clear {filename}: {fe}[/]")
    except Exception as e:
        console.print(f"[yellow]Warning: Could not clear logs directory: {e}[/]")

    # Register signals for graceful exit
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # ZMQ setup
    zmq_context = zmq.Context()
    pub_socket = zmq_context.socket(zmq.PUB)
    pub_socket.setsockopt(zmq.CONFLATE, 1)
    # Using a standard port for local pub/sub
    try:
        pub_socket.bind("tcp://127.0.0.1:5555")
    except Exception as e:
        console.print(f"[bold red]ZMQ Bind Error (Mode Pub):[//] {e}")

    # SUB to inference for RC commands
    rc_sub = zmq_context.socket(zmq.SUB)
    rc_sub.setsockopt(zmq.CONFLATE, 1)  # Keep only the last message
    rc_sub.connect("tcp://127.0.0.1:5556")
    rc_sub.setsockopt_string(zmq.SUBSCRIBE, "")

    shm_name = "drone_sensor_data"
    shm_size = 7 * 8
    
    hb_shm_name = "system_heartbeats"
    hb_shm_size = 3 * 8

    # Try to connect to SHM, if not exists, use zeros
    sensor_data = np.zeros(7, dtype=np.float64)
    heartbeats = np.zeros(3, dtype=np.float64)
    rc_commands = [1000, 1000, 1000, 1000, 1000, 1000]

    # Save terminal settings to restore later
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Set terminal to raw mode to capture single key presses
        tty.setcbreak(sys.stdin.fileno())
        
        # Initial publish
        publish_mode_cmd(pub_socket)
        
        # Start keyboard thread
        kb_thread = threading.Thread(target=keyboard_listener, args=(pub_socket,), daemon=True)
        kb_thread.start()
        
        with Live(generate_dashboard(sensor_data, heartbeats, current_mode, rc_commands, target_altitude), console=console, screen=True, refresh_per_second=10) as live:
            shm_mgr = None
            hb_shm_mgr = None
            try:
                global shutting_down
                while not shutting_down:
                    # 1. Non-blocking read RC commands
                    try:
                        new_rc = rc_sub.recv_pyobj(flags=zmq.NOBLOCK)
                        if new_rc:
                            rc_commands = new_rc
                    except zmq.Again:
                        pass

                    # 2. Always try to connect if we don't have a manager
                    if shm_mgr is None:
                        try:
                            shm_mgr = SharedMemoryManager(shm_name, shm_size, create=False)
                        except Exception:
                            shm_mgr = None
                    
                    if shm_mgr:
                        try:
                            sensor_data = shm_mgr.read_array(np.float64, (7,))
                        except Exception:
                            if shm_mgr:
                                shm_mgr.close()
                            shm_mgr = None 

                    if hb_shm_mgr is None:
                        try:
                            hb_shm_mgr = SharedMemoryManager(hb_shm_name, hb_shm_size, create=False)
                        except Exception:
                            hb_shm_mgr = None
                    
                    if hb_shm_mgr:
                        try:
                            heartbeats = hb_shm_mgr.read_array(np.float64, (3,))
                            # If heartbeats are stale, the service might have restarted and recreated the SHM.
                            # We force a reconnect after a short grace period (2.0s).
                            if heartbeats[0] > 0 and (time.time() - heartbeats[0]) > 2.0:
                                hb_shm_mgr.close()
                                hb_shm_mgr = None
                                if shm_mgr:
                                    shm_mgr.close()
                                    shm_mgr = None
                        except Exception:
                            if hb_shm_mgr:
                                hb_shm_mgr.close()
                            hb_shm_mgr = None
                    
                    with mode_lock:
                        live.update(generate_dashboard(sensor_data, heartbeats, current_mode, rc_commands, target_altitude))
                    time.sleep(0.1)
                
                # Show shutdown message
                with mode_lock:
                    live.update(generate_dashboard(sensor_data, heartbeats, current_mode, rc_commands, target_altitude, is_exiting=True))
                time.sleep(0.8)
            finally:
                if shm_mgr:
                    shm_mgr.close()
                if hb_shm_mgr:
                    hb_shm_mgr.close()
    except Exception as e:
        console.print(f"[bold red]Error:[/] {e}")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        pub_socket.close()
        rc_sub.close()
        zmq_context.term()
    
    console.print("[bold green]Dashboard exited gracefully.[/]")

if __name__ == "__main__":
    main()
