"""Arctos Bridge Node — communication bridge between STM32 and ROS2.

This is the main node that orchestrates all communication between the STM32 firmware
and the ROS2 ecosystem. It manages:
  - UDP command sending to STM32 (port 8888)
  - UDP state broadcast receiving from STM32 (port 8889)
  - ROS2 topic publishing (/arctos/state, /arctos/endstops)
  - ROS2 topic subscription (/arctos/cmd_positions, /arctos/cmd_servo)
  - ROS2 services (ping, stop, homing, servo control, etc.)
  - ROS2 action server (FollowJointTrajectory)

The node is structured into logical modules:
  - core: Parameter management, publishers, subscribers
  - services: Service handlers and service manager
  - communication: Broadcast receiver and state publisher
  - protocol: STM32 protocol client and unit conversion
  - actions: Trajectory action server
"""

import threading
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from arctos_msgs.msg import ConnectionStatus

from arctos_bridge.core import BridgeParameters, BridgePublishers, BridgeSubscribers
from arctos_bridge.services import ServiceHandlers, ServiceManager
from arctos_bridge.communication import BroadcastReceiver, ConnectionMonitor, StatePublisher
from arctos_bridge.protocol import STM32CommandClient, UnitConverter
from arctos_bridge.actions import TrajectoryActionServer


class ArctossBridgeNode(Node):
    """Main bridge node connecting STM32 firmware to ROS2.
    
    This node acts as a bidirectional bridge:
      - Receives state broadcasts from STM32 → publishes to ROS2 topics
      - Receives ROS2 commands → forwards to STM32 via UDP
      - Provides services for direct STM32 control
      - Provides action server for trajectory execution
    
    Attributes:
        params: Bridge parameters (loaded from ROS2 parameter server)
        _bridge_publishers: ROS2 publishers for state and endstop data
        subscribers: ROS2 subscribers for command topics
        service_manager: Manager for all ROS2 services
        broadcast_receiver: UDP broadcast receiver thread
        trajectory_action: FollowJointTrajectory action server
    """
    
    def __init__(self) -> None:
        """Initialize the Arctos bridge node.
        
        Sets up all components in the following order:
          1. Load parameters
          2. Initialize unit converter
          3. Initialize command client
          4. Initialize publishers and subscribers
          5. Initialize services
          6. Initialize trajectory action server
          7. Start broadcast receiver
        """
        super().__init__("arctos_bridge_node")
        
        # Thread-safe state tracking
        self._last_system_state: int = 0
        self._state_lock = threading.Lock()
        self._trajectory_active: bool = False
        self._trajectory_active_lock = threading.Lock()
        
        # Initialize all components
        self._init_parameters()
        self._init_unit_converter()
        self._init_command_client()
        self._init_publishers()
        self._init_subscribers()
        self._init_connection_monitor()
        self._init_services()
        self._init_trajectory_action()
        self._init_state_publisher()
        self._init_broadcast_receiver()
        
        self.get_logger().info("Arctos bridge node initialized successfully")
    
    def _init_parameters(self) -> None:
        """Load and validate all ROS2 parameters."""
        self.params = BridgeParameters(self)
    
    def _init_unit_converter(self) -> None:
        """Initialize the unit converter for steps ↔ radians conversion."""
        self._converter = UnitConverter(
            self.params.steps_per_rev,
            self.params.microsteps,
            self.params.gear_ratios,
            self.params.joint_inversions,
        )
    
    def _init_command_client(self) -> None:
        """Initialize and connect the STM32 command client."""
        self._cmd_client = STM32CommandClient(
            self.params.stm32_host,
            self.params.command_port,
            self.params.socket_timeout,
        )
        self._cmd_client.connect()
        self.get_logger().info(
            f"Command client connected to {self.params.stm32_host}:{self.params.command_port}"
        )
    
    def _init_publishers(self) -> None:
        """Initialize all ROS2 publishers."""
        self._bridge_publishers = BridgePublishers(self)
        self._connection_status_pub = self.create_publisher(
            ConnectionStatus, "/arctos/connection_status", 10
        )
    
    def _init_subscribers(self) -> None:
        """Initialize all ROS2 subscribers with their callbacks."""
        self.subscribers = BridgeSubscribers(
            self,
            self._handle_cmd_positions,
            self._handle_cmd_servo,
        )
    
    def _init_connection_monitor(self) -> None:
        """Initialize the connection monitor for automatic reconnection."""
        self._connection_monitor = ConnectionMonitor(
            node=self,
            timeout_s=self.params.connection_timeout,
            initial_delay_s=self.params.reconnect_initial_delay,
            max_delay_s=self.params.reconnect_max_delay,
            backoff_multiplier=self.params.reconnect_backoff_multiplier,
            auto_reconnect=self.params.auto_reconnect_enabled,
            reconnect_fn=self._attempt_reconnect,
            status_publish_fn=self._connection_status_pub.publish,
        )
    
    def _init_services(self) -> None:
        """Initialize all ROS2 services."""
        handlers = ServiceHandlers(
            self, self._cmd_client, self._converter, self._connection_monitor,
        )
        self.service_manager = ServiceManager(self, handlers)
    
    def _init_trajectory_action(self) -> None:
        """Initialize the FollowJointTrajectory action server."""
        self._traj_action = TrajectoryActionServer(
            node=self,
            cmd_client=self._cmd_client,
            converter=self._converter,
            joint_names=list(self.params.joint_names),
            get_system_state_fn=self._get_system_state,
        )
        self.get_logger().info("Trajectory action server initialized")
    
    def _init_state_publisher(self) -> None:
        """Initialize the state message publisher."""
        self._state_publisher = StatePublisher(
            self,
            self._converter,
            self.params.num_joints,
        )
    
    def _init_broadcast_receiver(self) -> None:
        """Initialize and start the UDP broadcast receiver."""
        self._broadcast_receiver = BroadcastReceiver(
            self,
            self._cmd_client,
            self.params.listen_port,
            self.params.broadcast_rate_hz,
            self._on_state_received,
        )
    
    # ------------------------------------------------------------------ #
    # Command topic callbacks                                            #
    # ------------------------------------------------------------------ #
    
    def _handle_cmd_positions(self, msg: Float64MultiArray) -> None:
        """Handle position commands from hardware interface/servo controller.
        
        Converts joint positions from radians to steps and forwards to STM32
        as CMD_MOVE_ALL. Blocked during trajectory execution.
        
        Args:
            msg: Float64MultiArray with 6 joint positions in radians
        """
        with self._trajectory_active_lock:
            if self._trajectory_active:
                return
        
        if len(msg.data) != self.params.num_joints:
            return
        
        positions_steps = [
            self._converter.rad_to_steps(msg.data[i], i)
            for i in range(self.params.num_joints)
        ]
        
        try:
            self._cmd_client.move_all(
                positions_steps,
                self.params.servo_move_velocity,
                self.params.servo_move_accel,
            )
        except Exception as e:
            self.get_logger().warn(f"CMD_MOVE_ALL failed: {e}")
    
    def _handle_cmd_servo(self, msg: Float64MultiArray) -> None:
        """Handle gripper servo commands.
        
        Forwards servo PWM commands to STM32 as CMD_SET_SERVO.
        
        Args:
            msg: Float64MultiArray with [pulse_us, duration_ms]
        """
        if len(msg.data) < 2:
            return
        
        pulse_us = int(msg.data[0])
        duration_ms = int(msg.data[1])
        
        try:
            self._cmd_client.set_servo(pulse_us, duration_ms)
        except Exception as e:
            self.get_logger().warn(f"CMD_SET_SERVO failed: {e}")
    
    # ------------------------------------------------------------------ #
    # Broadcast state handling                                            #
    # ------------------------------------------------------------------ #
    
    def _on_state_received(self, state: Dict[str, Any]) -> None:
        """Callback for received state broadcasts from STM32.
        
        Updates internal state tracking and publishes ROS2 messages.
        
        Args:
            state: Parsed state dictionary from broadcast receiver
        """
        self._connection_monitor.notify_received()
        
        with self._state_lock:
            self._last_system_state = state["system_state"]
        
        state_msg = self._state_publisher.create_state_message(state)
        self._bridge_publishers.publish_state(state_msg)
        
        endstop_msg = self._state_publisher.create_endstop_message(state)
        self._bridge_publishers.publish_endstops(endstop_msg)
    
    def _get_system_state(self) -> int:
        """Get the last received system state (thread-safe).
        
        Used by the trajectory action server to monitor execution.
        
        Returns:
            Current system state code (0=IDLE, 3=TRAJ_RUNNING, etc.)
        """
        with self._state_lock:
            return self._last_system_state
    
    # ------------------------------------------------------------------ #
    # Public properties for trajectory action server                     #
    # ------------------------------------------------------------------ #
    
    @property
    def command_client(self) -> STM32CommandClient:
        """Expose the command client for extensions.
        
        Returns:
            The STM32 command client instance
        """
        return self._cmd_client
    
    @property
    def converter(self) -> UnitConverter:
        """Expose the unit converter for extensions.
        
        Returns:
            The unit converter instance
        """
        return self._converter
    
    # ------------------------------------------------------------------ #
    # Shutdown                                                            #
    # ------------------------------------------------------------------ #
    
    def _attempt_reconnect(self) -> bool:
        """Try to re-establish the STM32 connection.
        
        Sends a ping to verify the STM32 is alive, then resubscribes
        to state broadcasts.
        
        Returns:
            True if both ping and resubscribe succeeded.
        """
        try:
            if not self._cmd_client.ping():
                return False
        except Exception:
            return False
        
        try:
            success = self._cmd_client.subscribe(
                self.params.listen_port, self.params.broadcast_rate_hz,
            )
            return success
        except Exception:
            return False
    
    def destroy_node(self) -> None:
        """Clean shutdown of all components.
        
        Stops the connection monitor, broadcast receiver, closes the
        command client, and calls the parent destroy_node().
        """
        self.get_logger().info("Shutting down arctos_bridge_node")
        
        self._connection_monitor.shutdown()
        self._broadcast_receiver.shutdown()
        
        try:
            self._cmd_client.close()
        except Exception:
            pass
        
        super().destroy_node()


def main(args=None) -> None:
    """Main entry point for the arctos_bridge_node.
    
    Initializes ROS2, creates the node, and spins until shutdown.
    
    Args:
        args: Command-line arguments (optional)
    """
    rclpy.init(args=args)
    node = ArctossBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
