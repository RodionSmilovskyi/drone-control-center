import time
import os
import numpy as np
import subprocess
import signal
import sys
import select
import tty
import termios
from rich.console import Console
from rich.layout import Layout
from rich.panel import Panel
from rich.live import Live
from rich.text import Text
from rich.table import Table
from core.shared_memory_manager import SharedMemoryManager

# Global flag for shutdown
shutting_down = False

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

    # Indices: 0: Sensors, 1: Strategic Agent, 2: FC
    service_map = [
        ("Sensors:", heartbeats[0]),
        ("Strategic agent:", heartbeats[1]),
        ("Flight controller:", heartbeats[2]),
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
    
    labels = ["Altitude", "Shift X", "Shift Y", "Velocity X", "Velocity Y"]
    for i, label in enumerate(labels):
        val = data[i] if i < len(data) else 0.0
        table.add_row(label, f"{val: .3f}")
    
    # Show heartbeat age
    heartbeat = data[5] if len(data) > 5 else 0.0
    diff = time.time() - heartbeat if heartbeat > 0 else 0.0
    table.add_section()
    table.add_row("Heartbeat age", f"[cyan]{diff: .2f}s[/cyan]")
    
    return table

def generate_dashboard(sensor_data: np.ndarray, heartbeats: np.ndarray, is_exiting: bool = False) -> Panel:
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
    
    sensors_ok = heartbeats[0] > 0 and (time.time() - heartbeats[0]) < 1.0
    
    status_block = Panel(
        status_table,
        title="[bold]Statuses[/bold]",
        border_style="green" if sensors_ok else "red",
        expand=False
    )
    
    # Create a layout for the internal content
    inner_layout = Layout()
    inner_layout.split_row(
        Layout(status_block, size=35),
        Layout(Panel(sensor_table, border_style="cyan"), name="center")
    )
    
    return Panel(
        inner_layout,
        title="[bold blue]Drone Control Center[/bold blue]",
        border_style="bright_blue",
        expand=True
    )

def is_key_pressed():
    """Returns True if there is a key waiting in stdin."""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main():
    console = Console()
    
    # Register signals for graceful exit
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    shm_name = "drone_sensor_data"
    shm_size = 6 * 8
    
    hb_shm_name = "system_heartbeats"
    hb_shm_size = 3 * 8

    # Try to connect to SHM, if not exists, use zeros
    sensor_data = np.zeros(6, dtype=np.float64)
    heartbeats = np.zeros(3, dtype=np.float64)

    # Save terminal settings to restore later
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Set terminal to raw mode to capture single key presses
        tty.setcbreak(sys.stdin.fileno())
        
        with Live(generate_dashboard(sensor_data, heartbeats), console=console, screen=True, refresh_per_second=10) as live:
            shm_mgr = None
            hb_shm_mgr = None
            try:
                global shutting_down
                while not shutting_down:
                    # Check for keyboard input
                    if is_key_pressed():
                        char = sys.stdin.read(1)
                        if char.lower() == 'q':
                            shutting_down = True
                            break

                    # Always try to connect if we don't have a manager
                    if shm_mgr is None:
                        try:
                            shm_mgr = SharedMemoryManager(shm_name, shm_size, create=False)
                        except Exception:
                            shm_mgr = None
                    
                    if shm_mgr:
                        try:
                            sensor_data = shm_mgr.read_array(np.float64, (6,))
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
                    
                    live.update(generate_dashboard(sensor_data, heartbeats))
                    time.sleep(0.1)
                
                # Show shutdown message
                live.update(generate_dashboard(sensor_data, heartbeats, is_exiting=True))
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
    
    console.print("[bold green]Dashboard exited gracefully.[/]")

if __name__ == "__main__":
    main()
