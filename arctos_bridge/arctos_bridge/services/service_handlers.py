"""Service request handlers for the Arctos bridge node.

This module contains all service callback implementations that handle ROS2 service
requests and forward them to the STM32 via the command client.
"""

import time
from typing import Any
from rclpy.node import Node

from arctos_msgs.srv import (
    Ping, Stop, SetServo, HomeAxis,
    HomeToPosition, SetHomeOffset, GetHomeOffset, JogAxis,
    SetAxisPosition, Reconnect,
)

from arctos_bridge.communication.connection_monitor import ConnectionMonitor
from arctos_bridge.protocol.stm32_protocol import STM32CommandClient
from arctos_bridge.protocol.unit_conversion import UnitConverter


class ServiceHandlers:
    """Handles all ROS2 service requests for the Arctos bridge.
    
    This class implements the callback functions for all services provided by
    the bridge node. Each handler converts ROS2 service requests into STM32
    protocol commands and returns appropriate responses.
    """
    
    def __init__(
        self,
        node: Node,
        cmd_client: STM32CommandClient,
        converter: UnitConverter,
        connection_monitor: ConnectionMonitor,
    ) -> None:
        """Initialize service handlers.
        
        Args:
            node: ROS2 node for logging
            cmd_client: STM32 command client for sending commands
            converter: Unit converter for radians ↔ steps conversion
            connection_monitor: Connection monitor for reconnection
        """
        self._node = node
        self._cmd_client = cmd_client
        self._converter = converter
        self._connection_monitor = connection_monitor
    
    def handle_ping(
        self,
        request: Ping.Request,
        response: Ping.Response,
    ) -> Ping.Response:
        """Handle ping service request.
        
        Sends a ping command to the STM32 and measures round-trip time.
        
        Args:
            request: Empty ping request
            response: Response with success flag and round-trip time
            
        Returns:
            Response with success=True if ping succeeded, round_trip_ms contains latency
        """
        try:
            t0 = time.monotonic()
            success = self._cmd_client.ping()
            dt = (time.monotonic() - t0) * 1000.0
            response.success = success
            response.round_trip_ms = dt
        except Exception as e:
            self._node.get_logger().error(f"Ping failed: {e}")
            response.success = False
            response.round_trip_ms = 0.0
        return response
    
    def handle_stop(
        self,
        request: Stop.Request,
        response: Stop.Response,
    ) -> Stop.Response:
        """Handle stop service request.
        
        Sends emergency stop command to the STM32, halting all motion.
        
        Args:
            request: Empty stop request
            response: Response with success flag and message
            
        Returns:
            Response with success=True if stop succeeded
        """
        try:
            success, msg = self._cmd_client.stop()
            response.success = success
            response.message = msg
        except Exception as e:
            self._node.get_logger().error(f"Stop failed: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def handle_set_servo(
        self,
        request: SetServo.Request,
        response: SetServo.Response,
    ) -> SetServo.Response:
        """Handle set_servo service request.
        
        Sets the gripper servo PWM pulse width and duration.
        
        Args:
            request: Request with pulse_us (500-2500) and duration_ms
            response: Response with success flag and message
            
        Returns:
            Response with success=True if servo command succeeded
        """
        try:
            success, msg = self._cmd_client.set_servo(
                request.pulse_us, request.duration_ms
            )
            response.success = success
            response.message = msg
        except Exception as e:
            self._node.get_logger().error(f"SetServo failed: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def handle_home_axis(
        self,
        request: HomeAxis.Request,
        response: HomeAxis.Response,
    ) -> HomeAxis.Response:
        """Handle home_axis service request.
        
        Starts or stops homing for a single axis.
        
        Args:
            request: Request with axis, direction, velocity, and start flag
            response: Response with success flag and message
            
        Returns:
            Response with success=True if homing command succeeded
        """
        try:
            if request.start:
                velocity_steps = abs(
                    self._converter.rad_per_sec_to_steps_per_sec(
                        request.velocity, request.axis
                    )
                )
                success, msg = self._cmd_client.home_start(
                    request.axis, request.direction, velocity_steps
                )
            else:
                success, msg = self._cmd_client.home_stop()
            response.success = success
            response.message = msg
        except Exception as e:
            self._node.get_logger().error(f"HomeAxis failed: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def handle_home_to_position(
        self,
        request: HomeToPosition.Request,
        response: HomeToPosition.Response,
    ) -> HomeToPosition.Response:
        """Handle home_to_position service request.
        
        Homes an axis to its endstop, then moves to a specified offset position.
        
        Args:
            request: Request with axis, velocity, and offset_steps
            response: Response with success flag and message
            
        Returns:
            Response with success=True if command succeeded
        """
        try:
            velocity_steps = abs(
                self._converter.rad_per_sec_to_steps_per_sec(
                    request.velocity, request.axis
                )
            ) if request.velocity > 0.0 else 0.0
            success, msg = self._cmd_client.home_to_position(
                request.axis, velocity_steps, request.offset_steps
            )
            response.success = success
            response.message = msg
        except Exception as e:
            self._node.get_logger().error(f"HomeToPosition failed: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def handle_set_home_offset(
        self,
        request: SetHomeOffset.Request,
        response: SetHomeOffset.Response,
    ) -> SetHomeOffset.Response:
        """Handle set_home_offset service request.
        
        Configures the home offset and endstop direction for an axis.
        
        Args:
            request: Request with axis, direction (0=MIN, 1=MAX), and offset_steps
            response: Response with success flag and message
            
        Returns:
            Response with success=True if offset was set
        """
        try:
            success, msg = self._cmd_client.set_home_offset(
                request.axis, request.direction, request.offset_steps
            )
            response.success = success
            response.message = msg
        except Exception as e:
            self._node.get_logger().error(f"SetHomeOffset failed: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def handle_get_home_offset(
        self,
        request: GetHomeOffset.Request,
        response: GetHomeOffset.Response,
    ) -> GetHomeOffset.Response:
        """Handle get_home_offset service request.
        
        Retrieves the configured home offset for an axis.
        
        Args:
            request: Request with axis index
            response: Response with success, offset_steps, and message
            
        Returns:
            Response with success=True and offset_steps if query succeeded
        """
        try:
            success, offset, msg = self._cmd_client.get_home_offset(
                request.axis
            )
            response.success = success
            response.offset_steps = offset
            response.message = msg
        except Exception as e:
            self._node.get_logger().error(f"GetHomeOffset failed: {e}")
            response.success = False
            response.offset_steps = 0
            response.message = str(e)
        return response
    
    def handle_jog_axis(
        self,
        request: JogAxis.Request,
        response: JogAxis.Response,
    ) -> JogAxis.Response:
        """Handle jog_axis service request.
        
        Jogs a single axis by a relative distance.
        
        Args:
            request: Request with axis, distance_steps, and velocity
            response: Response with success flag and message
            
        Returns:
            Response with success=True if jog command succeeded
        """
        try:
            velocity_steps = abs(
                self._converter.rad_per_sec_to_steps_per_sec(
                    request.velocity, request.axis
                )
            )
            accel_steps = velocity_steps * 2.0
            success, msg = self._cmd_client.jog_axis(
                request.axis, request.distance_steps,
                velocity_steps, accel_steps,
            )
            response.success = success
            response.message = msg
        except Exception as e:
            self._node.get_logger().error(f"JogAxis failed: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def handle_set_axis_position(
        self,
        request: SetAxisPosition.Request,
        response: SetAxisPosition.Response,
    ) -> SetAxisPosition.Response:
        """Handle set_axis_position service request.
        
        Sets the step counter for a single axis to an arbitrary value.
        Useful for resetting position to zero after homing.
        
        Args:
            request: Request with axis and position (steps)
            response: Response with success flag and message
            
        Returns:
            Response with success=True if position was set
        """
        try:
            success, msg = self._cmd_client.set_axis_position(
                request.axis, request.position
            )
            response.success = success
            response.message = msg
        except Exception as e:
            self._node.get_logger().error(f"SetAxisPosition failed: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def handle_reconnect(
        self,
        request: Reconnect.Request,
        response: Reconnect.Response,
    ) -> Reconnect.Response:
        """Handle reconnect service request.
        
        Triggers an immediate reconnection attempt to the STM32.
        
        Args:
            request: Empty reconnect request
            response: Response with success flag, message, and status
            
        Returns:
            Response with success=True if reconnection succeeded
        """
        try:
            success = self._connection_monitor.force_reconnect()
            response.success = success
            response.status = self._connection_monitor.status
            response.message = "Reconnected" if success else "Reconnection failed"
        except Exception as e:
            self._node.get_logger().error(f"Reconnect failed: {e}")
            response.success = False
            response.status = self._connection_monitor.status
            response.message = str(e)
        return response
