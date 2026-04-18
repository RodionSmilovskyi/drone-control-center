import time
from rich.console import Console
from rich.layout import Layout
from rich.panel import Panel
from rich.live import Live
from rich.text import Text
from rich.table import Table

def create_layout() -> Layout:
    """Define the dashboard layout."""
    layout = Layout()
    
    # We create a main split, but the user specifically asked for a status block at top left
    # inside a rectangular frame.
    layout.split_column(
        Layout(name="main")
    )
    return layout

def get_status_table() -> Table:
    """Creates the status block with stub data."""
    table = Table.grid(padding=(0, 1))
    table.add_column(style="bold white")
    table.add_column()

    statuses = [
        ("Sensors:", "Ok"),
        ("Flight controller:", "Ok"),
        ("Tactical controller:", "Ok"),
        ("Strategic agent:", "OK"),
    ]

    for label, status in statuses:
        table.add_row(label, f"[bold green]{status}[/bold green]")
    
    return table

def generate_dashboard() -> Panel:
    """Generates the full dashboard panel."""
    # The user wants a rectangular frame with a status block at the top left.
    status_table = get_status_table()
    
    # Wrap status table in its own panel for that "block" look
    status_block = Panel(
        status_table,
        title="[bold]Statuses[/bold]",
        border_style="green",
        expand=False # To keep it to the top-left
    )
    
    return Panel(
        status_block,
        title="[bold blue]Drone Control Center[/bold blue]",
        border_style="bright_blue",
        expand=True
    )

def main():
    console = Console()
    
    with Live(generate_dashboard(), console=console, screen=True, refresh_per_second=4) as live:
        try:
            while True:
                # Update the dashboard
                live.update(generate_dashboard())
                time.sleep(0.25)
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()
