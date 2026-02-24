"""Parameter management for the Arctos bridge node.

This module handles declaration, loading, and validation of all ROS2 parameters
used by the arctos_bridge node.
"""

from typing import List
from rclpy.node import Node


class BridgeParameters:
    """Manages all parameters for the Arctos bridge node.
    
    Attributes:
        stm32_host: IP address of the STM32 board
        command_port: UDP port for commands (default: 8888)
        broadcast_port: UDP port for state broadcasts from STM32 (default: 8889)
        listen_port: Local UDP port to receive broadcasts (default: 9000)
        broadcast_rate_hz: Requested broadcast rate from STM32 (1-200 Hz)
        socket_timeout: Socket timeout in seconds
        steps_per_rev: Motor steps per revolution (without microstepping)
        microsteps: Microstepping factor
        joint_names: List of joint names (6 joints)
        gear_ratios: Gear ratio for each joint
        joint_inversions: Direction inversion flag for each joint
        servo_move_velocity: Default velocity for servo moves (steps/s)
        servo_move_accel: Default acceleration for servo moves (steps/s²)
        num_joints: Number of joints (derived from joint_names)
    """
    
    def __init__(self, node: Node) -> None:
        """Initialize and load all parameters from the ROS2 node.
        
        Args:
            node: ROS2 node instance to declare and get parameters from
        """
        self._node = node
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()
    
    def _declare_parameters(self) -> None:
        """Declare all ROS2 parameters with default values."""
        self._node.declare_parameter("stm32_host", "192.168.178.159")
        self._node.declare_parameter("stm32_command_port", 8888)
        self._node.declare_parameter("stm32_broadcast_port", 8889)
        self._node.declare_parameter("broadcast_listen_port", 9000)
        self._node.declare_parameter("broadcast_rate_hz", 100)
        self._node.declare_parameter("protocol_version", 2)
        self._node.declare_parameter("socket_timeout_s", 2.0)
        self._node.declare_parameter("steps_per_rev", 200)
        self._node.declare_parameter("microsteps", 16)
        self._node.declare_parameter(
            "joint_names",
            ["X_joint", "Y_joint", "Z_joint", "A_joint", "B_joint", "C_joint"],
        )
        self._node.declare_parameter(
            "gear_ratios", [13.5, 150.0, 150.0, 48.0, 27.3375, 10.0]
        )
        self._node.declare_parameter(
            "joint_inversions", [False, False, False, True, False, True]
        )
        self._node.declare_parameter("servo_move_velocity", 50000.0)
        self._node.declare_parameter("servo_move_accel", 150000.0)
        self._node.declare_parameter("connection_timeout_s", 3.0)
        self._node.declare_parameter("reconnect_initial_delay_s", 0.5)
        self._node.declare_parameter("reconnect_max_delay_s", 5.0)
        self._node.declare_parameter("reconnect_backoff_multiplier", 1.5)
        self._node.declare_parameter("auto_reconnect_enabled", True)
    
    def _load_parameters(self) -> None:
        """Load parameter values from the ROS2 parameter server."""
        self.stm32_host: str = self._node.get_parameter("stm32_host").value
        self.command_port: int = self._node.get_parameter("stm32_command_port").value
        self.broadcast_port: int = self._node.get_parameter("stm32_broadcast_port").value
        self.listen_port: int = self._node.get_parameter("broadcast_listen_port").value
        self.broadcast_rate_hz: int = self._node.get_parameter("broadcast_rate_hz").value
        self.socket_timeout: float = self._node.get_parameter("socket_timeout_s").value
        self.steps_per_rev: int = self._node.get_parameter("steps_per_rev").value
        self.microsteps: int = self._node.get_parameter("microsteps").value
        self.joint_names: List[str] = self._node.get_parameter("joint_names").value
        self.gear_ratios: List[float] = self._node.get_parameter("gear_ratios").value
        self.joint_inversions: List[bool] = self._node.get_parameter("joint_inversions").value
        self.servo_move_velocity: float = self._node.get_parameter("servo_move_velocity").value
        self.servo_move_accel: float = self._node.get_parameter("servo_move_accel").value
        self.connection_timeout: float = self._node.get_parameter("connection_timeout_s").value
        self.reconnect_initial_delay: float = self._node.get_parameter("reconnect_initial_delay_s").value
        self.reconnect_max_delay: float = self._node.get_parameter("reconnect_max_delay_s").value
        self.reconnect_backoff_multiplier: float = self._node.get_parameter("reconnect_backoff_multiplier").value
        self.auto_reconnect_enabled: bool = self._node.get_parameter("auto_reconnect_enabled").value
        self.num_joints: int = len(self.joint_names)
    
    def _validate_parameters(self) -> None:
        """Validate parameter values and log warnings for invalid configurations."""
        if self.num_joints != 6:
            self._node.get_logger().warn(
                f"Expected 6 joints, got {self.num_joints}. System may not work correctly."
            )
        
        if len(self.gear_ratios) != self.num_joints:
            raise ValueError(
                f"gear_ratios length ({len(self.gear_ratios)}) must match "
                f"joint_names length ({self.num_joints})"
            )
        
        if len(self.joint_inversions) != self.num_joints:
            raise ValueError(
                f"joint_inversions length ({len(self.joint_inversions)}) must match "
                f"joint_names length ({self.num_joints})"
            )
        
        if not (1 <= self.broadcast_rate_hz <= 200):
            self._node.get_logger().warn(
                f"broadcast_rate_hz={self.broadcast_rate_hz} outside recommended range [1, 200]"
            )
        
        self._node.get_logger().info(
            f"Parameters loaded: {self.num_joints} joints, "
            f"STM32 @ {self.stm32_host}:{self.command_port}"
        )
