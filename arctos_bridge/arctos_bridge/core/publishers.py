"""ROS2 publishers for the Arctos bridge node.

This module manages all ROS2 publishers that send data from the STM32 to the ROS2 ecosystem.
"""

from rclpy.node import Node
from arctos_msgs.msg import ArctosState, ArctosEndstops


class BridgePublishers:
    """Manages all ROS2 publishers for the Arctos bridge.
    
    Publishers:
        - /arctos/state: ArctosState messages with full system state
        - /arctos/endstops: ArctosEndstops messages with endstop status
    """
    
    def __init__(self, node: Node) -> None:
        """Initialize all publishers.
        
        Args:
            node: ROS2 node instance to create publishers on
        """
        self._node = node
        
        self.state_pub = self._node.create_publisher(
            ArctosState, "/arctos/state", 10
        )
        
        self.endstop_pub = self._node.create_publisher(
            ArctosEndstops, "/arctos/endstops", 10
        )
        
        self._node.get_logger().info("Publishers initialized")
    
    def publish_state(self, msg: ArctosState) -> None:
        """Publish an ArctosState message.
        
        Args:
            msg: The state message to publish
        """
        self.state_pub.publish(msg)
    
    def publish_endstops(self, msg: ArctosEndstops) -> None:
        """Publish an ArctosEndstops message.
        
        Args:
            msg: The endstop message to publish
        """
        self.endstop_pub.publish(msg)
