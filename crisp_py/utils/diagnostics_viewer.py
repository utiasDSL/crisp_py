"""Simple terminal diagnostics viewer using rich."""

import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rich.live import Live
from rich.table import Table


class DiagnosticViewer:
    """A simple diagnostics viewer using rich."""

    def __init__(self):
        """Initialize the diagnostics viewer."""
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("diagnostic_viewer")
        self.node.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._callback_diagnostics,
            10,
        )
        self._update_timer = self.node.create_timer(1.0, self._update_table)

        self._diagnostics_aggregated = DiagnosticArray()
        self._diagnostics_timestamps = {}

        self._diagnostics_table = Table()

        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        """Spin the node to process callbacks."""
        while rclpy.ok():
            rclpy.spin_once(self.node)

    def _callback_diagnostics(self, msg: DiagnosticArray):
        """Callback for diagnostics messages."""
        self.node.get_logger().debug("Received diagnostics message.")
        current_time = time.time()

        for status in msg.status:
            names = [s.name for s in self._diagnostics_aggregated.status]
            if status.name not in names:
                self._diagnostics_aggregated.status.append(status)
                self._diagnostics_timestamps[status.name] = current_time
            else:
                for existing_status in self._diagnostics_aggregated.status:
                    if existing_status.name == status.name:
                        existing_status.level = status.level
                        existing_status.message = status.message
                        existing_status.hardware_id = status.hardware_id
                        existing_status.values = status.values
                        self._diagnostics_timestamps[status.name] = current_time
                        break

    def _update_table(self):
        """Update the diagnostics table with random data for demonstration."""
        self.node.get_logger().debug("Updating diagnostics table.")
        if not self._diagnostics_aggregated.status:
            return
        self._diagnostics_table = Table(title="Diagnostics Viewer", show_lines=True)
        self._diagnostics_table.add_column("Name", justify="left", style="cyan")
        self._diagnostics_table.add_column("Status", justify="left", style="green")
        self._diagnostics_table.add_column("Message", justify="left", style="yellow")
        self._diagnostics_table.add_column("Value", justify="left", style="bold")
        self._diagnostics_table.add_column("Age", justify="right", style="dim")

        current_time = time.time()
        for status in self._diagnostics_aggregated.status:
            name = status.name if status.name else "Unknown"
            level = status.level
            if level == DiagnosticStatus.OK:
                status_str = "[green]OK[/green]"
            elif level == DiagnosticStatus.WARN:
                status_str = "[yellow]WARNING[/yellow]"
            elif level == DiagnosticStatus.ERROR:
                status_str = "[red]ERROR[/red]"
            else:
                status_str = "[red]STALE[/red]"
            message = status.message if status.message else "No message"

            timestamp = self._diagnostics_timestamps.get(name, current_time)
            age_seconds = current_time - timestamp
            if age_seconds < 60:
                age_str = f"{age_seconds:.1f}s"
            elif age_seconds < 3600:
                age_str = f"{age_seconds / 60:.1f}m"
            else:
                age_str = f"{age_seconds / 3600:.1f}h"

            # Warn if the age is more than 5 seconds
            if age_seconds > 5:
                status_str = "[yellow]NOT ACTIVE[/yellow]"
                age_str = "[yellow]" + age_str + "[/yellow]"

            self._diagnostics_table.add_row(
                name,
                status_str,
                message,
                " | ".join(f"{kv.key}: {kv.value}" for kv in status.values),
                age_str,
            )

        if not self._diagnostics_table:
            self._diagnostics_table = Table(title="No Diagnostics Data Available")
            self._diagnostics_table.add_column("Name", justify="left", style="cyan")
            self._diagnostics_table.add_column("Status", justify="left", style="green")
            self._diagnostics_table.add_column("Message", justify="left", style="yellow")

    def display(self):
        """Display the table."""
        with Live(self._diagnostics_table, refresh_per_second=1) as live:
            while rclpy.ok():
                live.update(self._diagnostics_table)
                time.sleep(0.5)
