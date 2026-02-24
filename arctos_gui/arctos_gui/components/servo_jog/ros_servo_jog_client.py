"""ROS 2 servo jog client wrapping ``arctos_servo.ServoClient``."""

from __future__ import annotations

import logging
from typing import Dict, Sequence, Tuple

from arctos_servo import ServoClient

from .servo_jog_client_protocol import ServoJogClient

_LOG = logging.getLogger(__name__)


class ArctosRosServoJogClient(ServoJogClient):
    """Thin adapter that delegates to :class:`arctos_servo.ServoClient`.

    ``ServoClient`` manages its own ROS node, executor thread, controller
    switching, and MoveIt Servo lifecycle — this class simply exposes the
    subset of its API that the widget needs.
    """

    def __init__(self, node_name: str = "arctos_gui_servo_jog") -> None:
        """Initialize the client.

        Args:
            node_name: ROS node name used by the underlying ``ServoClient``.
        """
        self._client = ServoClient(
            node_name=node_name,
            auto_switch_controllers=True,
        )

    def connect(self) -> bool:
        """Start MoveIt Servo and switch to the servo controller."""
        ok = self._client.connect()
        if ok:
            _LOG.info("Servo jog client connected")
        else:
            _LOG.error("Servo jog client failed to connect")
        return ok

    def disconnect(self) -> None:
        """Stop MoveIt Servo and switch back to the trajectory controller."""
        self._client.disconnect()
        _LOG.info("Servo jog client disconnected")

    def is_connected(self) -> bool:
        """Return True while the servo backend is ready for commands."""
        return self._client.is_connected()

    def joint_jog(self, joint_name: str, velocity: float) -> None:
        """Publish a single-joint velocity command."""
        self._client.joint_jog(joint_name, velocity)

    def multi_joint_jog(
        self,
        joint_names: Sequence[str],
        velocities: Sequence[float],
    ) -> None:
        """Publish velocity commands for multiple joints simultaneously."""
        self._client.multi_joint_jog(joint_names, velocities)

    def cartesian_jog(
        self,
        linear: Tuple[float, float, float],
        angular: Tuple[float, float, float],
    ) -> None:
        """Publish a cartesian twist command."""
        self._client.cartesian_jog(linear=linear, angular=angular)

    def stop(self) -> None:
        """Send zero-velocity commands on both channels."""
        self._client.stop()

    def get_joint_positions(self) -> Dict[str, float]:
        """Return the latest joint positions as ``{name: radians}``."""
        return self._client.get_joint_positions()

    def get_ee_pose(self) -> Dict[str, float]:
        """Return the current end-effector pose."""
        return self._client.get_ee_pose()
