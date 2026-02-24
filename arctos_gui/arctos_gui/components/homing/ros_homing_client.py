"""ROS 2 client for homing operations.

Provides service clients for:
  - /arctos/home_axis          (HomeAxis)
  - /arctos/home_to_position   (HomeToPosition)
  - /arctos/jog_axis           (JogAxis)
  - /arctos/stop               (Stop)

Subscribes to /arctos/state for live homing status.
Exposes data via the HomingClient protocol.
"""

from __future__ import annotations

import logging
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from arctos_msgs.msg import ArctosState
from arctos_msgs.srv import (
    HomeAxis,
    HomeToPosition,
    JogAxis,
    Stop,
    SetAxisPosition,
    SetHomeOffset,
)

from .homing_client_protocol import HomingAxisStatus, HomingStatus

logger = logging.getLogger(__name__)

_TOPIC_STATE = "/arctos/state"
_SPIN_TIMEOUT_S = 0.05

_HOMING_STATE_NAMES = {
    0: "IDLE",
    1: "SEEKING",
    2: "RETURNING",
    3: "COMPLETE",
    4: "ERROR",
}

_SERVICE_TIMEOUT_S = 5.0


class ArctosRosHomingClient:
    """ROS 2 backend for the homing widget.

    Creates its own internal node and executor thread. Service calls
    are synchronous and thread-safe.
    """

    def __init__(self, node_name: str = "arctos_homing_client") -> None:
        """Initializes the ROS node, service clients, and state subscriber.

        Args:
            node_name: Name for the internal ROS node.
        """
        self._node: Node = rclpy.create_node(node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._lock = threading.Lock()
        self._connected = False
        self._homing_status = HomingStatus()
        self._positions: list[float] = [0.0] * 6
        self._positions_steps: list[float] = [0.0] * 6

        self._home_axis_cli = self._node.create_client(
            HomeAxis, "/arctos/home_axis",
        )
        self._home_to_pos_cli = self._node.create_client(
            HomeToPosition, "/arctos/home_to_position",
        )
        self._stop_cli = self._node.create_client(
            Stop, "/arctos/stop",
        )
        self._jog_cli = self._node.create_client(
            JogAxis, "/arctos/jog_axis",
        )
        self._set_axis_pos_cli = self._node.create_client(
            SetAxisPosition, "/arctos/set_axis_position",
        )
        self._set_home_offset_cli = self._node.create_client(
            SetHomeOffset, "/arctos/set_home_offset",
        )

        self._state_sub = self._node.create_subscription(
            ArctosState, _TOPIC_STATE, self._on_state, 10,
        )

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        """Spin the ROS executor loop."""
        while rclpy.ok():
            self._executor.spin_once(timeout_sec=_SPIN_TIMEOUT_S)

    def _on_state(self, msg: ArctosState) -> None:
        """Handle the state event."""
        status = HomingStatus()
        status.any_active = bool(msg.homing_active)
        status.active_axis = int(msg.homing_axis) if msg.homing_active else -1
        status.active_state = _HOMING_STATE_NAMES.get(
            msg.homing_state, str(msg.homing_state),
        )
        status.is_homed_bitmask = int(msg.is_homed_bitmask)

        for i in range(len(status.axes)):
            is_homed = bool(status.is_homed_bitmask & (1 << i))
            if status.any_active and i == status.active_axis:
                status.axes[i] = HomingAxisStatus(
                    state=status.active_state, active=True, is_homed=is_homed,
                )
            else:
                status.axes[i].is_homed = is_homed

        positions = list(msg.positions) if msg.positions else [0.0] * 6
        positions_steps = (
            list(msg.positions_steps) if msg.positions_steps else [0.0] * 6
        )

        with self._lock:
            self._homing_status = status
            self._connected = True
            self._positions = positions
            self._positions_steps = positions_steps

    def home_axis_to_endstop(
        self, axis: int, direction: int, velocity_rad_s: float,
    ) -> tuple[bool, str]:
        """Start homing a single axis to its endstop."""
        req = HomeAxis.Request()
        req.axis = axis
        req.direction = direction
        req.velocity = velocity_rad_s
        req.start = True
        return self._call_home_axis(req)

    def stop_homing(self) -> tuple[bool, str]:
        """Stop all active homing operations."""
        req = Stop.Request()
        future = self._stop_cli.call_async(req)
        return self._wait_for_future(future)

    def home_axis_to_position(
        self, axis: int, velocity_rad_s: float, offset_steps: int,
    ) -> tuple[bool, str]:
        """Home axis to endstop then move to offset position."""
        req = HomeToPosition.Request()
        req.axis = axis
        req.velocity = velocity_rad_s
        req.offset_steps = offset_steps
        future = self._home_to_pos_cli.call_async(req)
        return self._wait_for_future(future)

    def get_homing_status(self) -> HomingStatus:
        """Returns the current homing status from the state broadcast."""
        with self._lock:
            return self._homing_status

    def get_current_position(self, axis: int) -> float:
        """Returns the current position of an axis in radians."""
        with self._lock:
            if 0 <= axis < len(self._positions):
                return self._positions[axis]
            return 0.0

    def get_current_position_steps(self, axis: int) -> float:
        """Returns the current position of an axis in steps."""
        with self._lock:
            if 0 <= axis < len(self._positions_steps):
                return self._positions_steps[axis]
            return 0.0

    def jog_axis(
        self, axis: int, distance_steps: int, velocity_rad_s: float,
    ) -> tuple[bool, str]:
        """Jog an axis by a relative distance in steps."""
        req = JogAxis.Request()
        req.axis = axis
        req.distance_steps = distance_steps
        req.velocity = velocity_rad_s
        future = self._jog_cli.call_async(req)
        return self._wait_for_future(future)

    def set_axis_position(self, axis: int, position_steps: float) -> tuple[bool, str]:
        """Set the step counter for a single axis to an arbitrary value."""
        req = SetAxisPosition.Request()
        req.axis = axis
        req.position = position_steps
        future = self._set_axis_pos_cli.call_async(req)
        return self._wait_for_future(future)

    def set_home_offset(
        self, axis: int, direction: int, offset_steps: int,
    ) -> tuple[bool, str]:
        """Set the home offset and direction for an axis."""
        req = SetHomeOffset.Request()
        req.axis = axis
        req.direction = direction
        req.offset_steps = offset_steps
        future = self._set_home_offset_cli.call_async(req)
        return self._wait_for_future(future)

    def is_connected(self) -> bool:
        """Returns True once at least one state message has been received."""
        with self._lock:
            return self._connected

    def disconnect(self) -> None:
        """Destroys the ROS node."""
        try:
            self._node.destroy_node()
        except Exception:
            logger.exception("Error destroying homing client node")

    def _call_home_axis(self, req: HomeAxis.Request) -> tuple[bool, str]:
        """Perform call home axis."""
        future = self._home_axis_cli.call_async(req)
        return self._wait_for_future(future)

    def _wait_for_future(self, future) -> tuple[bool, str]:
        """Wait for a service future to complete without spinning the executor."""
        start_time = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start_time > _SERVICE_TIMEOUT_S:
                return False, "Service call timed out"
            time.sleep(0.01)
        if future.result() is None:
            return False, "Service call failed"
        resp = future.result()
        return resp.success, resp.message
