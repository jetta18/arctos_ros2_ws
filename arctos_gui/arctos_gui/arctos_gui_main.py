"""Main entry point for the Arctos GUI."""

import os
import signal

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from .components.jog.ros_jog_client import ArctosRosJogClient
from .main import ArctosMainWindow
from .ui import apply_app_theme

try:
    from .components.cartesian_jog.ros_cartesian_jog_client import ArctosCartesianJogClient
    CARTESIAN_JOG_AVAILABLE = True
except ImportError:
    CARTESIAN_JOG_AVAILABLE = False


def main() -> None:
    """Main entry point for the Arctos GUI.
    
    Starts the Qt event loop with ROS-based jog client.
    """
    import sys

    rclpy.init(args=None)

    node_prefix = f"arctos_gui_{os.getpid()}"

    # Create ROS-based jog client
    jog_client = ArctosRosJogClient(node_name=f"{node_prefix}_jog_client")

    # Create MoveIt-based cartesian jog client (optional)
    cartesian_jog_client = None
    if CARTESIAN_JOG_AVAILABLE:
        try:
            cartesian_jog_client = ArctosCartesianJogClient(
                node_name=f"{node_prefix}_cartesian_jog_client"
            )
        except Exception as exc:
            print(f"Warning: Could not create cartesian jog client: {exc}")

    # Create Qt application
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)
    apply_app_theme(app)

    shutdown_started = False
    window = None

    def _shutdown() -> None:
        nonlocal shutdown_started
        if shutdown_started:
            return
        shutdown_started = True

        try:
            if window is not None:
                window.close()
            jog_client.disconnect()
            if cartesian_jog_client:
                cartesian_jog_client.disconnect()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    app.aboutToQuit.connect(_shutdown)

    def _handle_termination_signal(_signum: int, _frame) -> None:  # noqa: ANN001
        app.quit()

    signal.signal(signal.SIGINT, _handle_termination_signal)
    signal.signal(signal.SIGTERM, _handle_termination_signal)

    # Keep Python signal handling responsive while Qt event loop is running.
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(100)

    window = ArctosMainWindow(
        jog_client=jog_client,
        cartesian_jog_client=cartesian_jog_client,
    )
    window.show()

    exit_code = app.exec_()
    _shutdown()

    sys.exit(exit_code)
