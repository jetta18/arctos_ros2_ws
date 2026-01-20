"""Main entry point for the Arctos GUI."""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from .components.jog import EthernetJogClient
from .components.mks_config import MKSConfigClient
from .main import ArctosMainWindow


def main() -> None:
    """Main entry point for the Arctos GUI.
    
    Starts the Qt event loop with Ethernet clients and a ROS node for MKS config.
    """
    import sys

    rclpy.init(args=None)

    # Create clients
    jog_client = EthernetJogClient()
    mks_config_client = MKSConfigClient()  

    # ROS executor for MKS config client
    # executor = SingleThreadedExecutor()
    # executor.add_node(mks_config_client)

    # Create Qt application
    app = QApplication(sys.argv)
    window = ArctosMainWindow(
        jog_client=jog_client,
        mks_config_client=mks_config_client
    )
    window.show()

    # Periodically spin ROS executor
    # timer = QTimer()
    # timer.setInterval(10)  # ms
    # timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0.0))
    # timer.start()

    try:
        exit_code = app.exec_()
    finally:
        # executor.shutdown()
        # mks_config_client.destroy_node()
        rclpy.shutdown()
        # Disconnect Ethernet clients
        jog_client.disconnect()

    sys.exit(exit_code)
