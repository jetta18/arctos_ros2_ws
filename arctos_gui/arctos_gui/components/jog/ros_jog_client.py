"""ROS2 Jog Client implementing the JogClient protocol for the JogWidget."""

import threading
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from .jog_client_protocol import JogClient


class ArctosRosJogClient(JogClient):
    """
    ROS2-based jog client that talks to arctos_position_controller.
    
    Implements the JogClient protocol expected by JogWidget.
    """

    def __init__(self):
        # Joint configuration (must match ros2_controllers.yaml)
        self.joint_names = [
            "X_joint", "Y_joint", "Z_joint",
            "A_joint", "B_joint", "C_joint"
        ]
        self.current_positions = [0.0] * len(self.joint_names)

        # ROS infrastructure
        self._ros_node = None
        self._ros_executor = None
        self._ros_thread = None
        self._cmd_pub = None
        self._connected = False

        # Initialize ROS
        self._init_ros()

    def _init_ros(self):
        """Initialize ROS node, executor, and start background thread."""
        if not rclpy.ok():
            rclpy.init()

        self._ros_node = rclpy.create_node('arctos_jog_client')

        # Publisher for joint commands
        self._cmd_pub = self._ros_node.create_publisher(
            Float64MultiArray,
            '/arctos_position_controller/commands',
            10
        )

        # Subscriber for joint states (to keep positions in sync)
        self._ros_node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        # Executor and thread
        self._ros_executor = MultiThreadedExecutor()
        self._ros_executor.add_node(self._ros_node)

        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()
        self._connected = True

    def _ros_spin(self):
        """Spin the ROS executor in the background thread."""
        try:
            self._ros_executor.spin()
        except Exception as e:
            print(f"[ArctosRosJogClient] ROS spin error: {e}")

    def _joint_state_callback(self, msg: JointState):
        """Update current positions from /joint_states."""
        name_to_pos = dict(zip(msg.name, msg.position))
        for i, name in enumerate(self.joint_names):
            if name in name_to_pos:
                self.current_positions[i] = name_to_pos[name]

    # -----------------------------------------------------------------
    # Implementation of JogClient protocol
    # -----------------------------------------------------------------
    def send_jog(self, axis_index: int, delta_rad: float, velocity_rad_s: float) -> None:
        """
        Jog a single joint by delta_rad.
        
        Note: velocity_rad_s is ignored for the simple position controller.
        """
        if not self._connected:
            raise RuntimeError("ROS client not connected")

        if axis_index < 0 or axis_index >= len(self.current_positions):
            raise ValueError(f"Invalid axis_index {axis_index}")

        # Update local copy
        self.current_positions[axis_index] += delta_rad

        # Publish new command
        msg = Float64MultiArray()
        msg.data = self.current_positions
        self._cmd_pub.publish(msg)

    def connect(self) -> bool:
        """Connect to ROS (already done in __init__)."""
        return self._connected

    def disconnect(self) -> None:
        """Shutdown ROS node and thread."""
        self._connected = False
        if self._ros_executor:
            self._ros_executor.shutdown()
        if self._ros_node:
            self._ros_node.destroy_node()
        if self._ros_thread and self._ros_thread.is_alive():
            self._ros_thread.join(timeout=1.0)

    def is_connected(self) -> bool:
        """Return connection status."""
        return self._connected
