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
MAX_ALTITUDE = 3.0
SHIFT_Z_STEP = round(0.05 / MAX_ALTITUDE, 3)  # ~0.017 (+5cm per step)
SHIFT_XY_STEP = 0.05  # +0.05m (+5cm per step)
DEFAULT_SHIFT_Z = round(0.4 / MAX_ALTITUDE, 3)  # +0.133 (+0.40m default vertical shift)
desired_shift_z = DEFAULT_SHIFT_Z
desired_shift_x = 0.0
desired_shift_y = 0.0
mode_lock = threading.Lock()

# Indoor-safe maneuver test sequence with smooth setpoint ramping:
# Ramp rates smoothly lead vertical and horizontal maneuvers without abrupt derivative spikes.
TEST_Z_RAMP_RATE_MS = 0.10   # 10 cm/s vertical setpoint ramp rate
TEST_XY_RAMP_RATE_MS = 0.12  # 12 cm/s horizontal setpoint ramp rate (matches 0.09 m/s cruise speed without overshoot)
# (step_label, mode, shift_z_meters, shift_x_meters, shift_y_meters, duration_seconds)
TEST_SEQUENCE_STEPS = [
    ("1/14 Arming Motors", "armed", 0.40, 0.00, 0.00, 1.5),
    ("2/14 Takeoff & Hover (0.40m)", "ai", 0.40, 0.00, 0.00, 4.5),
    ("3/14 Climb (0.40m -> 0.62m)", "ai", 0.62, 0.00, 0.00, 5.0),
    ("4/14 Descend (0.62m -> 0.28m)", "ai", 0.28, 0.00, 0.00, 5.5),
    ("5/14 Level Hover (0.40m)", "ai", 0.40, 0.00, 0.00, 4.0),
    ("6/14 Pitch Forward (+25cm Y)", "ai", 0.40, 0.00, 0.25, 8.0),
    ("7/14 Center Hover", "ai", 0.40, 0.00, 0.00, 6.0),
    ("8/14 Pitch Back (-25cm Y)", "ai", 0.40, 0.00, -0.25, 8.0),
    ("9/14 Center Hover", "ai", 0.40, 0.00, 0.00, 6.0),
    ("10/14 Roll Right (+25cm X)", "ai", 0.40, 0.25, 0.00, 8.0),
    ("11/14 Center Hover", "ai", 0.40, 0.00, 0.00, 6.0),
    ("12/14 Roll Left (-25cm X)", "ai", 0.40, -0.25, 0.00, 8.0),
    ("13/14 Final Hover (0.40m)", "ai", 0.40, 0.00, 0.00, 5.5),
    ("14/14 Soft Landing", "ai", 0.00, 0.00, 0.00, 4.5),
]
test_seq_active = False
test_seq_index = 0
test_step_start_time = 0.0
test_last_ramp_time = 0.0

def apply_test_step(step_idx: int, pub_socket):
    """Starts the given test sequence step; step<=1 latches initial hover setpoint, step>=2 ramps smoothly."""
    global current_mode, desired_shift_z, desired_shift_x, desired_shift_y, test_seq_index, test_step_start_time, test_last_ramp_time
    _, mode, sz_m, sx_m, sy_m, _ = TEST_SEQUENCE_STEPS[step_idx]
    now = time.time()
    test_seq_index = step_idx
    test_step_start_time = now
    test_last_ramp_time = now
    mode_changed = (current_mode != mode)
    current_mode = mode
    if step_idx <= 1:
        desired_shift_z = round(sz_m / MAX_ALTITUDE, 3)
        desired_shift_x = round(sx_m, 3)
        desired_shift_y = round(sy_m, 3)
        publish_mode_cmd(pub_socket)
    elif mode_changed:
        publish_mode_cmd(pub_socket)

def update_test_sequence_ramp(pub_socket):
    """Smoothly ramps desired_shift_{z,x,y} toward current step targets (caller holds mode_lock)."""
    global desired_shift_z, desired_shift_x, desired_shift_y, test_last_ramp_time
    now = time.time()
    dt = min(max(now - test_last_ramp_time, 0.0), 0.25)
    test_last_ramp_time = now
    if test_seq_index <= 1 or dt <= 0.0:
        return

    _, _, sz_m, sx_m, sy_m, _ = TEST_SEQUENCE_STEPS[test_seq_index]
    target_sz = sz_m / MAX_ALTITUDE
    target_sx = sx_m
    target_sy = sy_m

    max_dz = (TEST_Z_RAMP_RATE_MS / MAX_ALTITUDE) * dt
    max_dxy = TEST_XY_RAMP_RATE_MS * dt

    new_sz = round(desired_shift_z + float(np.clip(target_sz - desired_shift_z, -max_dz, max_dz)), 3)
    new_sx = round(desired_shift_x + float(np.clip(target_sx - desired_shift_x, -max_dxy, max_dxy)), 3)
    new_sy = round(desired_shift_y + float(np.clip(target_sy - desired_shift_y, -max_dxy, max_dxy)), 3)

    if new_sz != desired_shift_z or new_sx != desired_shift_x or new_sy != desired_shift_y:
        desired_shift_z = new_sz
        desired_shift_x = new_sx
        desired_shift_y = new_sy
        publish_mode_cmd(pub_socket)

def publish_mode_cmd(pub_socket):
    """Sends current flight mode and desired 3D shifts via non-blocking ZMQ."""
    payload = json.dumps({
        "mode": current_mode,
        "shift_z": round(desired_shift_z, 3),
        "shift_x": round(desired_shift_x, 3),
        "shift_y": round(desired_shift_y, 3),
    })
    try:
        pub_socket.send_string(payload, flags=zmq.NOBLOCK)
    except zmq.Again:
        pass

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

def get_mode_panel(
    mode: str,
    rc_commands: list,
    shift_z: float,
    shift_x: float = 0.0,
    shift_y: float = 0.0,
    test_status: str = "",
) -> Panel:
    """Creates a panel displaying the current operating mode, desired 3D shifts, test sequence state, and RC summary."""
    mode_colors = {
        "disarmed": "bold red",
        "armed": "bold green",
        "ai": "bold yellow"
    }
    color = mode_colors.get(mode, "bold white")
    
    mode_text = Text(f"{mode.upper()}", style=f"{color} underline")
    shift_z_text = Text(f"\nShift Z: {shift_z:+.3f} ({shift_z * MAX_ALTITUDE:+.2f}m)", style="bold cyan")
    shift_xy_text = Text(f"\nShift X/Y: {shift_x:+.2f}, {shift_y:+.2f}", style="cyan")
    if test_status:
        test_text = Text(f"\nTest Seq: {test_status}", style="bold magenta")
    else:
        test_text = Text("\nTest Seq: Idle (press 't' to run)", style="dim")
    controls_hint = Text("\na/d/x: Mode | t: Test Seq | w/s: Alt | i/k: Pitch | j/l: Roll", style="dim")
    rc_summary = Text(f"\nRC: {rc_commands[:4]}", style="dim")
    
    content = Text.assemble("\n", mode_text, "\n", shift_z_text, shift_xy_text, test_text, "\n", controls_hint, "\n", rc_summary)
    
    return Panel(content, title="[bold]System Status[/bold]", border_style=color.split()[-1], title_align="center")

def generate_dashboard(
    sensor_data: np.ndarray,
    heartbeats: np.ndarray,
    mode: str,
    rc_commands: list,
    shift_z: float = 0.133,
    shift_x: float = 0.0,
    shift_y: float = 0.0,
    test_status: str = "",
    is_exiting: bool = False,
) -> Panel:
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
    mode_panel = get_mode_panel(mode, rc_commands, shift_z, shift_x, shift_y, test_status)
    
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
    global current_mode, desired_shift_z, desired_shift_x, desired_shift_y, test_seq_active, shutting_down
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
            elif char_lower == 't':
                with mode_lock:
                    if not test_seq_active:
                        test_seq_active = True
                        apply_test_step(0, pub_socket)
                    else:
                        test_seq_active = False
                        desired_shift_z = DEFAULT_SHIFT_Z
                        desired_shift_x = 0.0
                        desired_shift_y = 0.0
                        publish_mode_cmd(pub_socket)
            elif char_lower == 'w':
                with mode_lock:
                    test_seq_active = False
                    desired_shift_z = min(1.0, round(desired_shift_z + SHIFT_Z_STEP, 3))
                    publish_mode_cmd(pub_socket)
            elif char_lower == 's':
                with mode_lock:
                    test_seq_active = False
                    desired_shift_z = max(-1.0, round(desired_shift_z - SHIFT_Z_STEP, 3))
                    publish_mode_cmd(pub_socket)
            elif char_lower == 'i':
                with mode_lock:
                    test_seq_active = False
                    desired_shift_y = min(1.0, round(desired_shift_y + SHIFT_XY_STEP, 2))
                    publish_mode_cmd(pub_socket)
            elif char_lower == 'k':
                with mode_lock:
                    test_seq_active = False
                    desired_shift_y = max(-1.0, round(desired_shift_y - SHIFT_XY_STEP, 2))
                    publish_mode_cmd(pub_socket)
            elif char_lower == 'j':
                with mode_lock:
                    test_seq_active = False
                    desired_shift_x = max(-1.0, round(desired_shift_x - SHIFT_XY_STEP, 2))
                    publish_mode_cmd(pub_socket)
            elif char_lower == 'l':
                with mode_lock:
                    test_seq_active = False
                    desired_shift_x = min(1.0, round(desired_shift_x + SHIFT_XY_STEP, 2))
                    publish_mode_cmd(pub_socket)
            elif char_lower == 'q':
                shutting_down = True
                break
            
            if new_mode:
                with mode_lock:
                    was_testing = test_seq_active
                    test_seq_active = False
                    if was_testing:
                        desired_shift_z = DEFAULT_SHIFT_Z
                        desired_shift_x = 0.0
                        desired_shift_y = 0.0
                    if current_mode != new_mode or was_testing:
                        current_mode = new_mode
                        publish_mode_cmd(pub_socket)
        except Exception:
            break

def main():
    global shutting_down, current_mode, desired_shift_z, desired_shift_x, desired_shift_y, test_seq_active
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
        
        with Live(generate_dashboard(sensor_data, heartbeats, current_mode, rc_commands, desired_shift_z, desired_shift_x, desired_shift_y), console=console, screen=True, refresh_per_second=10) as live:
            shm_mgr = None
            hb_shm_mgr = None
            try:
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
                        test_status = ""
                        if test_seq_active:
                            step_label, _, _, _, _, step_dur = TEST_SEQUENCE_STEPS[test_seq_index]
                            elapsed = time.time() - test_step_start_time
                            if elapsed >= step_dur:
                                if test_seq_index + 1 < len(TEST_SEQUENCE_STEPS):
                                    apply_test_step(test_seq_index + 1, pub_socket)
                                    step_label, _, _, _, _, step_dur = TEST_SEQUENCE_STEPS[test_seq_index]
                                    elapsed = 0.0
                                else:
                                    test_seq_active = False
                                    current_mode = "disarmed"
                                    desired_shift_z = DEFAULT_SHIFT_Z
                                    desired_shift_x = 0.0
                                    desired_shift_y = 0.0
                                    publish_mode_cmd(pub_socket)
                            if test_seq_active:
                                update_test_sequence_ramp(pub_socket)
                                rem = max(0.0, step_dur - elapsed)
                                test_status = f"{step_label} ({rem:.1f}s)"
                        live.update(generate_dashboard(sensor_data, heartbeats, current_mode, rc_commands, desired_shift_z, desired_shift_x, desired_shift_y, test_status))
                    time.sleep(0.1)
                
                # Show shutdown message
                with mode_lock:
                    live.update(generate_dashboard(sensor_data, heartbeats, current_mode, rc_commands, desired_shift_z, desired_shift_x, desired_shift_y, is_exiting=True))
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
