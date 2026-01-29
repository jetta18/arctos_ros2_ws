"""Main entry point for the Arctos GUI."""

import rclpy
from PyQt5.QtWidgets import QApplication

from .components.jog.ros_jog_client import ArctosRosJogClient
from .main import ArctosMainWindow
from .ui import apply_app_theme


def main() -> None:
    """Main entry point for the Arctos GUI.
    
    Starts the Qt event loop with ROS-based jog client.
    """
    import sys

    rclpy.init(args=None)

    # Create ROS-based jog client
    jog_client = ArctosRosJogClient()

    # Create Qt application
    app = QApplication(sys.argv)
    apply_app_theme(app)
    window = ArctosMainWindow(jog_client=jog_client)
    window.show()

    try:
        exit_code = app.exec_()
    finally:
        rclpy.shutdown()
        # Disconnect ROS client
        jog_client.disconnect()

    sys.exit(exit_code)
