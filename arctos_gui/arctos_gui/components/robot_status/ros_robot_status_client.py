"""ROS 2 client for robot status data.

Subscribes to:
  - /arctos/endstops  (arctos_msgs/ArctosEndstops) — per-axis endstop states
  - /arctos/state     (arctos_msgs/ArctosState)    — system state and diagnostics

Exposes data via the RobotStatusClient protocol.
"""

from __future__ import annotations

import logging
import threading
from typing import Any

import rclpy
from rclpy.node import Node

from arctos_msgs.msg import ArctosEndstops, ArctosState, ConnectionStatus
from arctos_msgs.srv import Reconnect

from ...backend.app_state import AXIS_NAMES
from .robot_status_client_protocol import (
    ConnectionData,
    DiagnosticData,
    EndstopAxisData,
    JointStateData,
    default_endstop_data,
    default_joint_data,
)

logger = logging.getLogger(__name__)

_TOPIC_ENDSTOPS = "/arctos/endstops"
_TOPIC_STATE = "/arctos/state"
_TOPIC_CONNECTION = "/arctos/connection_status"
_SERVICE_RECONNECT = "/arctos/reconnect"
_SPIN_TIMEOUT_S = 0.05

_AXIS_INDEX: dict[str, int] = {name: i for i, name in enumerate(AXIS_NAMES)}

_STATE_NAMES: dict[int, str] = {
    0: "IDLE",
    1: "MOVING",
    2: "TRAJ_LOADING",
    3: "TRAJ_RUNNING",
    4: "TRAJ_COMPLETE",
    5: "ERROR",
    6: "STOPPING",
}


class ArctosRosRobotStatusClient:
    """Subscribes to endstop and state topics and caches the latest data.

    The widget polls this client via the RobotStatusClient protocol.
    """

    def __init__(self, node_name: str = "arctos_robot_status_client") -> None:
        """Initializes the ROS node and subscribers.

        Args:
            node_name: Name for the internal ROS node.
        """
        self._node: Node = rclpy.create_node(node_name)
        self._lock = threading.Lock()
        self._endstop_data: dict[str, EndstopAxisData] = default_endstop_data()
        self._joint_data: dict[str, JointStateData] = default_joint_data()
        self._diagnostic_data = DiagnosticData()
        self._connection_data = ConnectionData()
        self._connected = False

        self._endstop_sub = self._node.create_subscription(
            ArctosEndstops,
            _TOPIC_ENDSTOPS,
            self._on_endstops,
            10,
        )
        self._state_sub = self._node.create_subscription(
            ArctosState,
            _TOPIC_STATE,
            self._on_state,
            10,
        )
        self._conn_sub = self._node.create_subscription(
            ConnectionStatus,
            _TOPIC_CONNECTION,
            self._on_connection_status,
            10,
        )
        self._reconnect_client = self._node.create_client(
            Reconnect, _SERVICE_RECONNECT,
        )

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self._node.get_logger().info(
            "ArctosRosRobotStatusClient started, listening on "
            f"{_TOPIC_ENDSTOPS}, {_TOPIC_STATE}, {_TOPIC_CONNECTION}"
        )

    def _spin(self) -> None:
        """Spin the ROS executor loop."""
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=_SPIN_TIMEOUT_S)

    def _on_endstops(self, msg: ArctosEndstops) -> None:
        """Handle the endstops event."""
        endstop_data = default_endstop_data()
        for axis, axis_idx in _AXIS_INDEX.items():
            min_bit = axis_idx * 2
            max_bit = axis_idx * 2 + 1
            min_count = msg.trigger_counts[min_bit] if min_bit < len(msg.trigger_counts) else 0
            max_count = msg.trigger_counts[max_bit] if max_bit < len(msg.trigger_counts) else 0
            endstop_data[axis] = EndstopAxisData(
                min_triggered=bool(msg.endstop_states & (1 << min_bit)),
                max_triggered=bool(msg.endstop_states & (1 << max_bit)),
                min_trigger_count=min_count,
                max_trigger_count=max_count,
            )

        with self._lock:
            self._endstop_data = endstop_data
            self._connected = True

    def _on_state(self, msg: ArctosState) -> None:
        """Handle the state event."""
        joint_data = default_joint_data()
        for axis, axis_idx in _AXIS_INDEX.items():
            is_homed = bool(msg.is_homed_bitmask & (1 << axis_idx))
            pos_rad = msg.positions[axis_idx] if axis_idx < len(msg.positions) else 0.0
            vel_rad_s = msg.velocities[axis_idx] if axis_idx < len(msg.velocities) else 0.0
            pos_steps = msg.positions_steps[axis_idx] if axis_idx < len(msg.positions_steps) else 0.0
            
            joint_data[axis] = JointStateData(
                position_rad=pos_rad,
                velocity_rad_s=vel_rad_s,
                position_steps=pos_steps,
                is_homed=is_homed,
            )
            
        diag_data = DiagnosticData(
            system_state=_STATE_NAMES.get(msg.system_state, str(msg.system_state)),
            uptime_s=msg.stm32_uptime_ms / 1000.0,
            servo_pulse_us=msg.servo_pulse_us,
            broadcast_seq=msg.broadcast_seq,
            homing_active=msg.homing_active,
            trajectory_id=msg.trajectory_id,
            traj_points_loaded=msg.traj_points_loaded,
            traj_current_segment=msg.traj_current_segment,
            traj_total_segments=msg.traj_total_segments,
        )

        with self._lock:
            self._joint_data = joint_data
            self._diagnostic_data = diag_data
            self._connected = True

    def _on_connection_status(self, msg: ConnectionStatus) -> None:
        """Handle connection status updates from the bridge."""
        conn = ConnectionData(
            status=msg.status,
            last_received_time=msg.last_received_time,
            reconnect_attempts=msg.reconnect_attempts,
            message=msg.message,
        )
        with self._lock:
            self._connection_data = conn

    def get_endstop_data(self) -> dict[str, EndstopAxisData]:
        """Returns the latest cached per-axis endstop states and trigger counts.

        Returns:
            Mapping of axis name to EndstopAxisData.
        """
        with self._lock:
            return {k: EndstopAxisData(**vars(v)) for k, v in self._endstop_data.items()}

    def get_joint_data(self) -> dict[str, JointStateData]:
        """Returns the latest cached per-axis joint states.

        Returns:
            Mapping of axis name to JointStateData.
        """
        with self._lock:
            return {k: JointStateData(**vars(v)) for k, v in self._joint_data.items()}

    def get_diagnostic_data(self) -> DiagnosticData:
        """Returns the latest cached system diagnostics.

        Returns:
            DiagnosticData containing system state info.
        """
        with self._lock:
            return DiagnosticData(**vars(self._diagnostic_data))

    def get_connection_data(self) -> ConnectionData:
        """Returns the latest STM32 connection status.

        Returns:
            ConnectionData with status, timestamps, and attempt count.
        """
        with self._lock:
            return ConnectionData(**vars(self._connection_data))

    def request_reconnect(self) -> tuple[bool, str]:
        """Call the /arctos/reconnect service synchronously.

        Returns:
            (success, message) tuple.
        """
        if not self._reconnect_client.wait_for_service(timeout_sec=2.0):
            return False, "Reconnect service not available"
        request = Reconnect.Request()
        future = self._reconnect_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if future.result() is None:
            return False, "Reconnect service call timed out"
        result = future.result()
        return result.success, result.message

    def is_connected(self) -> bool:
        """Returns True once at least one message has been received.

        Returns:
            True if data has been received.
        """
        with self._lock:
            return self._connected

    def disconnect(self) -> None:
        """Destroys the ROS node and stops the spin thread."""
        try:
            self._node.destroy_node()
        except Exception:
            logger.exception("Error destroying robot status node")
