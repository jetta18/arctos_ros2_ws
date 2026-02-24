"""Main entry point for the Arctos GUI."""

from __future__ import annotations

import logging
import os
import signal

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from .backend.settings_manager import SettingsManager
from .components.jog.ros_jog_client import ArctosRosJogClient
from .main import ArctosMainWindow
from .ui import apply_app_theme

logger = logging.getLogger(__name__)

try:
    from .components.servo_jog.ros_servo_jog_client import ArctosRosServoJogClient
    SERVO_JOG_AVAILABLE = True
except ImportError:
    SERVO_JOG_AVAILABLE = False


def main() -> None:
    """Main entry point for the Arctos GUI.

    Initialises settings, ROS clients, and the Qt application, then starts
    the event loop.
    """
    import sys

    logging.basicConfig(level=logging.INFO)

    settings_manager = SettingsManager()
    settings_manager.load()

    rclpy.init(args=None)

    node_prefix = f"arctos_gui_{os.getpid()}"

    jog_client = ArctosRosJogClient(node_name=f"{node_prefix}_jog_client")

    servo_jog_client = None
    if SERVO_JOG_AVAILABLE:
        try:
            servo_jog_client = ArctosRosServoJogClient(
                node_name=f"{node_prefix}_servo_jog"
            )
        except Exception as exc:
            logger.warning("Could not create servo jog client: %s", exc)

    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)
    apply_app_theme(app)

    shutdown_started = False
    window = None

    def _shutdown() -> None:
        """Close the window, disconnect clients, and shut down ROS exactly once."""
        nonlocal shutdown_started
        if shutdown_started:
            return
        shutdown_started = True

        try:
            if window is not None:
                window.close()
            jog_client.disconnect()
            if servo_jog_client:
                servo_jog_client.disconnect()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    app.aboutToQuit.connect(_shutdown)

    def _handle_termination_signal(_signum: int, _frame) -> None:  # noqa: ANN001
        """Trigger Qt shutdown when SIGINT/SIGTERM is received."""
        app.quit()

    signal.signal(signal.SIGINT, _handle_termination_signal)
    signal.signal(signal.SIGTERM, _handle_termination_signal)

    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(100)

    window = ArctosMainWindow(
        jog_client=jog_client,
        servo_jog_client=servo_jog_client,
        settings_manager=settings_manager,
    )
    window.show()

    exit_code = app.exec_()
    _shutdown()

    sys.exit(exit_code)
