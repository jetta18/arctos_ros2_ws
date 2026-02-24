"""ROS2 subscribers for the Arctos bridge node.

This module manages all ROS2 subscribers that receive commands from the ROS2 ecosystem
and forward them to the STM32.
"""

import threading
from typing import Callable
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class BridgeSubscribers:
    """Manages all ROS2 subscribers for the Arctos bridge.
    
    Subscribers:
        - /arctos/cmd_positions: Position commands from hardware interface/servo controller
        - /arctos/cmd_servo: Gripper servo commands
    """
    
    def __init__(
        self,
        node: Node,
        cmd_positions_callback: Callable[[Float64MultiArray], None],
        cmd_servo_callback: Callable[[Float64MultiArray], None],
    ) -> None:
        """Initialize all subscribers.
        
        Args:
            node: ROS2 node instance to create subscribers on
            cmd_positions_callback: Callback for position command messages
            cmd_servo_callback: Callback for servo command messages
        """
        self._node = node
        
        self.cmd_pos_sub = self._node.create_subscription(
            Float64MultiArray,
            "/arctos/cmd_positions",
            cmd_positions_callback,
            10,
        )
        
        self.cmd_servo_sub = self._node.create_subscription(
            Float64MultiArray,
            "/arctos/cmd_servo",
            cmd_servo_callback,
            10,
        )
        
        self._node.get_logger().info("Subscribers initialized")
