"""Service manager for the Arctos bridge node.

This module handles the initialization and registration of all ROS2 services.
"""

from rclpy.node import Node

from arctos_msgs.srv import (
    Ping, Stop, SetServo, HomeAxis,
    HomeToPosition, SetHomeOffset, GetHomeOffset, JogAxis,
    SetAxisPosition, Reconnect,
)

from .service_handlers import ServiceHandlers


class ServiceManager:
    """Manages all ROS2 services for the Arctos bridge.
    
    Creates and registers all service servers, connecting them to the
    appropriate handler methods.
    """
    
    def __init__(self, node: Node, handlers: ServiceHandlers) -> None:
        """Initialize all services.
        
        Args:
            node: ROS2 node instance to create services on
            handlers: Service handlers instance with callback methods
        """
        self._node = node
        self._handlers = handlers
        
        # Create all service servers
        self._ping_srv = self._node.create_service(
            Ping, "/arctos/ping", self._handlers.handle_ping
        )
        
        self._stop_srv = self._node.create_service(
            Stop, "/arctos/stop", self._handlers.handle_stop
        )
        
        self._servo_srv = self._node.create_service(
            SetServo, "/arctos/set_servo", self._handlers.handle_set_servo
        )
        
        self._home_srv = self._node.create_service(
            HomeAxis, "/arctos/home_axis", self._handlers.handle_home_axis
        )
        
        self._home_to_pos_srv = self._node.create_service(
            HomeToPosition, "/arctos/home_to_position",
            self._handlers.handle_home_to_position,
        )
        
        self._set_offset_srv = self._node.create_service(
            SetHomeOffset, "/arctos/set_home_offset",
            self._handlers.handle_set_home_offset,
        )
        
        self._get_offset_srv = self._node.create_service(
            GetHomeOffset, "/arctos/get_home_offset",
            self._handlers.handle_get_home_offset,
        )
        
        self._jog_axis_srv = self._node.create_service(
            JogAxis, "/arctos/jog_axis",
            self._handlers.handle_jog_axis,
        )
        
        self._set_axis_pos_srv = self._node.create_service(
            SetAxisPosition, "/arctos/set_axis_position",
            self._handlers.handle_set_axis_position,
        )
        
        self._reconnect_srv = self._node.create_service(
            Reconnect, "/arctos/reconnect",
            self._handlers.handle_reconnect,
        )
        
        self._node.get_logger().info("Services initialized (10 services)")
