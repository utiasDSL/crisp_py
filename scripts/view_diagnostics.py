"""Script to run diagnostics viewer for the robot."""

from rich.console import Console

from crisp_py.utils.diagnostics_viewer import DiagnosticViewer

console = Console()
console.clear()

viewer = DiagnosticViewer()
viewer.display()
