"""Main entry point for the Arctos GUI."""

import rclpy
from PyQt5.QtWidgets import QApplication

from .components.jog import EthernetJogClient
from .main import ArctosMainWindow


def main() -> None:
    """Main entry point for the Arctos GUI.
    
    Starts the Qt event loop with Ethernet client.
    MKS config now uses direct CAN communication.
    """
    import sys

    rclpy.init(args=None)

    # Create Ethernet client for jog control
    jog_client = EthernetJogClient()

    # Create Qt application (MKS config uses direct CAN, no ROS2 client needed)
    app = QApplication(sys.argv)
    window = ArctosMainWindow(jog_client=jog_client)
    window.show()

    try:
        exit_code = app.exec_()
    finally:
        rclpy.shutdown()
        # Disconnect Ethernet client
        jog_client.disconnect()

    sys.exit(exit_code)
