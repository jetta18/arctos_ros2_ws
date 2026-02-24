"""ROS lifecycle and executor management for arctos_servo."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


@dataclass
class RosHandles:
    """Container for active ROS entities used by ServoClient."""

    node: Node
    jog_pub: rclpy.publisher.Publisher
    twist_pub: rclpy.publisher.Publisher
    start_client: rclpy.client.Client
    stop_client: rclpy.client.Client
    tf_buffer: Buffer


class RosContext:
    """Manage ROS node, executor, and spin thread lifecycle."""

    def __init__(
        self,
        node_name: str,
        joint_state_cb: Callable[[JointState], None],
        joint_jog_topic: str,
        twist_topic: str,
        joint_states_topic: str,
        start_servo_service: str,
        stop_servo_service: str,
    ) -> None:
        self._node_name = node_name
        self._joint_state_cb = joint_state_cb
        self._joint_jog_topic = joint_jog_topic
        self._twist_topic = twist_topic
        self._joint_states_topic = joint_states_topic
        self._start_servo_service = start_servo_service
        self._stop_servo_service = stop_servo_service

        self._lock = threading.Lock()
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._handles: Optional[RosHandles] = None
        self._tf_listener: Optional[TransformListener] = None

    def start(self) -> None:
        """Start ROS node and executor thread if not already running."""
        with self._lock:
            if self._handles is not None:
                return

            if not rclpy.ok():
                rclpy.init()

            node = Node(self._node_name)
            jog_pub = node.create_publisher(JointJog, self._joint_jog_topic, 10)
            twist_pub = node.create_publisher(TwistStamped, self._twist_topic, 10)
            start_client = node.create_client(Trigger, self._start_servo_service)
            stop_client = node.create_client(Trigger, self._stop_servo_service)
            node.create_subscription(
                JointState,
                self._joint_states_topic,
                self._joint_state_cb,
                10,
            )

            tf_buffer = Buffer()
            self._tf_listener = TransformListener(tf_buffer, node)

            executor = MultiThreadedExecutor()
            executor.add_node(node)

            self._executor = executor
            self._handles = RosHandles(
                node=node,
                jog_pub=jog_pub,
                twist_pub=twist_pub,
                start_client=start_client,
                stop_client=stop_client,
                tf_buffer=tf_buffer,
            )

            self._spin_thread = threading.Thread(
                target=self._spin,
                daemon=True,
            )
            self._spin_thread.start()

    def stop(self) -> None:
        """Stop executor thread and destroy ROS node."""
        with self._lock:
            executor = self._executor
            spin_thread = self._spin_thread
            handles = self._handles
            self._executor = None
            self._spin_thread = None
            self._handles = None
            self._tf_listener = None

        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass

        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=2.0)

        if handles is not None:
            try:
                handles.node.destroy_node()
            except Exception:
                pass

    def get_handles(self) -> Optional[RosHandles]:
        """Return active ROS handles or None if not running."""
        with self._lock:
            return self._handles

    def _spin(self) -> None:
        executor = self._executor
        if executor is None:
            return
        try:
            executor.spin()
        except Exception:
            pass
