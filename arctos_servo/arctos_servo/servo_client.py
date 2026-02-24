"""Thread-safe MoveIt Servo client for the Arctos robot.

Provides joint jog, cartesian jog, state queries, and lifecycle
management.  Designed to be imported from any Python context (GUI,
script, ROS node) without requiring the caller to manage ROS internals.

Typical usage::

    from arctos_servo import ServoClient

    client = ServoClient()
    client.connect()

    client.joint_jog("X_joint", 0.5)
    client.cartesian_jog(linear=(0.3, 0.0, 0.0))
    client.stop()

    client.disconnect()
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .controller_switch import ControllerSwitchConfig, ControllerSwitcher
from .constants import (
    BASE_FRAME,
    EE_FRAME,
    JOINT_JOG_TOPIC,
    JOINT_NAMES,
    JOINT_STATES_TOPIC,
    NUM_JOINTS,
    PUBLISH_RATE_HZ,
    SERVO_CONTROLLER,
    START_SERVO_SERVICE,
    STOP_SERVO_SERVICE,
    TRAJECTORY_CONTROLLER,
    TWIST_TOPIC,
)
from .ros_context import RosContext
from .state_cache import JointStateCache, get_ee_pose

_LOG = logging.getLogger(__name__)

_ZERO_LINEAR: Tuple[float, float, float] = (0.0, 0.0, 0.0)
_ZERO_ANGULAR: Tuple[float, float, float] = (0.0, 0.0, 0.0)


class ServoClient:
    """High-level, thread-safe client for MoveIt Servo on the Arctos robot.

    All public methods are safe to call from any thread (Qt UI thread,
    asyncio loop, plain script, etc.).

    The client creates its own internal ROS node and executor thread so
    the caller does not need to manage ROS lifecycle.
    """

    def __init__(
        self,
        node_name: str = 'arctos_servo_client',
        auto_switch_controllers: bool = True,
    ) -> None:
        self._node_name = node_name
        self._auto_switch = auto_switch_controllers

        self._state_lock = threading.Lock()
        self._connected = False
        self._servo_started = False

        self._state_cache = JointStateCache(JOINT_NAMES)
        self._ros_context = RosContext(
            node_name=node_name,
            joint_state_cb=self._joint_state_cb,
            joint_jog_topic=JOINT_JOG_TOPIC,
            twist_topic=TWIST_TOPIC,
            joint_states_topic=JOINT_STATES_TOPIC,
            start_servo_service=START_SERVO_SERVICE,
            stop_servo_service=STOP_SERVO_SERVICE,
        )
        self._switcher = ControllerSwitcher(
            ControllerSwitchConfig(
                servo_controller=SERVO_CONTROLLER,
                trajectory_controller=TRAJECTORY_CONTROLLER,
            ),
            logger=_LOG,
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self, timeout_sec: float = 10.0) -> bool:
        """Initialise ROS, switch controllers, and start MoveIt Servo.

        Returns True on success.
        """
        with self._state_lock:
            if self._connected:
                return True

        try:
            self._ros_context.start()

            if self._auto_switch:
                self.switch_to_servo_controller()

            if not self._call_start_servo(timeout_sec):
                _LOG.error('Failed to start MoveIt Servo')
                self._cleanup()
                return False

            with self._state_lock:
                self._connected = True
                self._servo_started = True
            return True

        except Exception:
            _LOG.exception('connect() failed')
            self._cleanup()
            return False

    def disconnect(self) -> None:
        """Stop servo, optionally switch back to trajectory controller."""
        with self._state_lock:
            if not self._connected:
                return
            self._connected = False

        self._call_stop_servo()

        if self._auto_switch:
            self.switch_to_trajectory_controller()

        self._cleanup()

    def is_connected(self) -> bool:
        """Return True if the client is connected and servo is running."""
        with self._state_lock:
            return self._connected

    # ------------------------------------------------------------------
    # Joint jog
    # ------------------------------------------------------------------

    def joint_jog(self, joint_name: str, velocity: float) -> None:
        """Publish a single joint jog command.

        Args:
            joint_name: One of the JOINT_NAMES (e.g. ``'X_joint'``).
            velocity:   Unitless velocity in ``[-1.0, 1.0]``.
        """
        if joint_name not in JOINT_NAMES:
            raise ValueError(
                f'Unknown joint {joint_name!r}. '
                f'Valid: {JOINT_NAMES}'
            )
        self._publish_joint_jog([joint_name], [float(velocity)])

    def joint_jog_by_index(self, joint_index: int, velocity: float) -> None:
        """Publish a single joint jog command by index (0-5)."""
        if not 0 <= joint_index < NUM_JOINTS:
            raise ValueError(
                f'joint_index must be 0..{NUM_JOINTS - 1}, '
                f'got {joint_index}'
            )
        self.joint_jog(JOINT_NAMES[joint_index], velocity)

    def multi_joint_jog(
        self,
        joint_names: Sequence[str],
        velocities: Sequence[float],
    ) -> None:
        """Publish a jog command for multiple joints simultaneously."""
        if len(joint_names) != len(velocities):
            raise ValueError('joint_names and velocities must have equal length')
        for name in joint_names:
            if name not in JOINT_NAMES:
                raise ValueError(f'Unknown joint {name!r}')
        self._publish_joint_jog(list(joint_names), [float(v) for v in velocities])

    # ------------------------------------------------------------------
    # Cartesian jog
    # ------------------------------------------------------------------

    def cartesian_jog(
        self,
        linear: Tuple[float, float, float] = _ZERO_LINEAR,
        angular: Tuple[float, float, float] = _ZERO_ANGULAR,
        frame_id: str = BASE_FRAME,
    ) -> None:
        """Publish a cartesian twist command to MoveIt Servo.

        Args:
            linear:  ``(x, y, z)`` unitless velocities in ``[-1.0, 1.0]``.
            angular: ``(rx, ry, rz)`` unitless velocities in ``[-1.0, 1.0]``.
            frame_id: Reference frame for the twist (default: base_link).
        """
        self._publish_twist(linear, angular, frame_id)

    # ------------------------------------------------------------------
    # Stop
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Send zero-velocity commands on both joint and cartesian channels."""
        self._publish_joint_jog(
            list(JOINT_NAMES), [0.0] * NUM_JOINTS,
        )
        self._publish_twist(_ZERO_LINEAR, _ZERO_ANGULAR, BASE_FRAME)

    # ------------------------------------------------------------------
    # State queries
    # ------------------------------------------------------------------

    def get_joint_positions(self) -> Dict[str, float]:
        """Return the latest joint positions as ``{name: radians}``."""
        return self._state_cache.snapshot()

    def get_joint_position(self, joint_name: str) -> float:
        """Return the latest position of a single joint in radians."""
        return self._state_cache.get_position(joint_name)

    def get_ee_pose(self) -> Dict[str, float]:
        """Return the current end-effector pose.

        Returns:
            Dict with keys ``x, y, z`` (metres) and ``rx, ry, rz`` (radians).
        """
        handles = self._ros_context.get_handles()
        tf_buffer = handles.tf_buffer if handles else None
        return get_ee_pose(tf_buffer, BASE_FRAME, EE_FRAME)

    # ------------------------------------------------------------------
    # Internal: ROS lifecycle
    # ------------------------------------------------------------------

    def _cleanup(self) -> None:
        with self._state_lock:
            self._servo_started = False
        self._ros_context.stop()

    # ------------------------------------------------------------------
    # Internal: controller switching
    # ------------------------------------------------------------------

    def switch_to_servo_controller(self) -> bool:
        """Load (if needed) and activate the servo controller."""
        self._switcher.ensure_servo_loaded()
        return self._switcher.switch_to_servo()

    def switch_to_trajectory_controller(self) -> bool:
        """Switch back to the trajectory controller."""
        return self._switcher.switch_to_trajectory()

    # ------------------------------------------------------------------
    # Internal: servo service calls
    # ------------------------------------------------------------------

    def _call_start_servo(self, timeout_sec: float) -> bool:
        handles = self._ros_context.get_handles()
        if handles is None:
            return False

        if not handles.start_client.wait_for_service(timeout_sec=timeout_sec):
            _LOG.error('start_servo service not available')
            return False

        future = handles.start_client.call_async(Trigger.Request())
        if not self._wait_for_future(future, timeout_sec=5.0):
            _LOG.error('start_servo failed: timeout')
            return False

        if future.result() is not None and future.result().success:
            return True

        msg = future.result().message if future.result() else 'timeout'
        _LOG.error('start_servo failed: %s', msg)
        return False

    def _call_stop_servo(self) -> None:
        handles = self._ros_context.get_handles()
        if handles is None:
            return
        if handles.stop_client.service_is_ready():
            handles.stop_client.call_async(Trigger.Request())

    # ------------------------------------------------------------------
    # Internal: publishing
    # ------------------------------------------------------------------

    def _publish_joint_jog(
        self, names: list, velocities: list,
    ) -> None:
        handles = self._ros_context.get_handles()
        if handles is None:
            return
        msg = JointJog()
        msg.header.stamp = handles.node.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.joint_names = names
        msg.velocities = velocities
        handles.jog_pub.publish(msg)

    def _publish_twist(
        self,
        linear: Tuple[float, float, float],
        angular: Tuple[float, float, float],
        frame_id: str,
    ) -> None:
        handles = self._ros_context.get_handles()
        if handles is None:
            return
        msg = TwistStamped()
        msg.header.stamp = handles.node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.twist.linear.x = float(linear[0])
        msg.twist.linear.y = float(linear[1])
        msg.twist.linear.z = float(linear[2])
        msg.twist.angular.x = float(angular[0])
        msg.twist.angular.y = float(angular[1])
        msg.twist.angular.z = float(angular[2])
        handles.twist_pub.publish(msg)

    # ------------------------------------------------------------------
    # Internal: callbacks
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState) -> None:
        self._state_cache.update_from_msg(msg)

    def _wait_for_future(
        self,
        future: rclpy.task.Future,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()
