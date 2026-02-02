"""ROS 2 jog client implementing the `JogClient` protocol."""

from __future__ import annotations

import logging
import threading
from typing import Any, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .jog_client_protocol import JogClient


_LOGGER = logging.getLogger(__name__)


class ArctosRosJogClient(JogClient):
    """ROS 2-based jog client for the joint trajectory controller."""

    def __init__(self, node_name: str = "arctos_jog_client") -> None:
        self._node_name = str(node_name)

        self.joint_names = [
            "X_joint",
            "Y_joint",
            "Z_joint",
            "A_joint",
            "B_joint",
            "C_joint",
        ]
        self._current_positions = [0.0] * len(self.joint_names)
        self._state_lock = threading.Lock()

        self._ros_node: Optional[Node] = None
        self._ros_executor: Optional[MultiThreadedExecutor] = None
        self._ros_thread: Optional[threading.Thread] = None
        self._cmd_pub: Optional[Any] = None
        self._connected = False

        self.connect()

    def connect(self) -> bool:
        """Start the ROS node/executor if needed."""
        if self._connected:
            return True

        try:
            self._start_ros()
        except Exception:  # noqa: BLE001
            _LOGGER.exception("Failed to start ROS jog client")
            self._stop_ros()
            return False

        return self._connected

    def disconnect(self) -> None:
        """Stop the ROS node/executor."""
        self._stop_ros()

    def is_connected(self) -> bool:
        return self._connected

    def get_current_position(self, axis_index: int) -> float:
        if axis_index < 0 or axis_index >= len(self._current_positions):
            raise ValueError(f"Invalid axis_index {axis_index}")

        with self._state_lock:
            return float(self._current_positions[axis_index])

    def send_jog(self, axis_index: int, delta_rad: float, velocity_rad_s: float) -> None:
        """Jog a single joint by `delta_rad`.
        """
        pub = self._cmd_pub
        if not self._connected or pub is None:
            raise RuntimeError("ROS jog client not connected")

        if axis_index < 0 or axis_index >= len(self._current_positions):
            raise ValueError(f"Invalid axis_index {axis_index}")

        # Build a small trajectory for the joint_trajectory_controller.
        # We keep the other joints at their current positions.
        with self._state_lock:
            start_positions = list(self._current_positions)
            self._current_positions[axis_index] += float(delta_rad)
            target_positions = list(self._current_positions)

        speed = max(float(velocity_rad_s), 1e-6)
        duration_s = abs(float(delta_rad)) / speed
        # Avoid zero-duration trajectories (can be treated as a step).
        duration_s = max(duration_s, 0.05)
        sec = int(duration_s)
        nanosec = int((duration_s - sec) * 1e9)

        start_velocities = [0.0] * len(self.joint_names)
        start_velocities[axis_index] = speed if delta_rad >= 0.0 else -speed
        end_velocities = [0.0] * len(self.joint_names)

        msg = JointTrajectory()
        msg.joint_names = list(self.joint_names)

        pt0 = JointTrajectoryPoint()
        pt0.positions = start_positions
        pt0.velocities = start_velocities
        pt0.time_from_start.sec = 0
        pt0.time_from_start.nanosec = 0

        pt1 = JointTrajectoryPoint()
        pt1.positions = target_positions
        pt1.velocities = end_velocities
        pt1.time_from_start.sec = sec
        pt1.time_from_start.nanosec = nanosec

        msg.points = [pt0, pt1]

        pub.publish(msg)

    def _start_ros(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self._ros_node = rclpy.create_node(self._node_name)

        self._cmd_pub = self._ros_node.create_publisher(
            JointTrajectory,
            "/arctos_controller/joint_trajectory",
            10,
        )

        self._ros_node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        self._ros_executor = MultiThreadedExecutor()
        self._ros_executor.add_node(self._ros_node)

        self._ros_thread = threading.Thread(target=self._spin, daemon=True)
        self._ros_thread.start()
        self._connected = True

    def _stop_ros(self) -> None:
        self._connected = False

        if self._ros_executor is not None:
            try:
                self._ros_executor.shutdown()
            except Exception:  # noqa: BLE001
                pass

        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:  # noqa: BLE001
                pass

        if self._ros_thread is not None and self._ros_thread.is_alive():
            self._ros_thread.join(timeout=1.0)

        self._ros_thread = None
        self._ros_executor = None
        self._ros_node = None
        self._cmd_pub = None

    def _spin(self) -> None:
        executor = self._ros_executor
        if executor is None:
            return

        try:
            executor.spin()
        except Exception:  # noqa: BLE001
            _LOGGER.exception("ROS executor spin failed")

    def _joint_state_callback(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        with self._state_lock:
            for i, name in enumerate(self.joint_names):
                if name in name_to_pos:
                    self._current_positions[i] = float(name_to_pos[name])
