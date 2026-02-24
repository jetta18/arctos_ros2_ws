"""Protocol definition for servo jog clients used by the ServoJogWidget."""

from __future__ import annotations

from typing import Dict, Protocol, Sequence, Tuple


class ServoJogClient(Protocol):
    """Abstraction for streaming velocity-based jog commands.

    Keeps the Qt layer independent from MoveIt Servo specifics.
    All velocity arguments are unitless in the range ``[-1.0, 1.0]``.
    """

    def connect(self) -> bool:
        """Start MoveIt Servo and switch to the servo controller."""
        ...

    def disconnect(self) -> None:
        """Stop MoveIt Servo and switch back to the trajectory controller."""
        ...

    def is_connected(self) -> bool:
        """Return True while the servo backend is ready for commands."""
        ...

    def joint_jog(self, joint_name: str, velocity: float) -> None:
        """Publish a single-joint velocity command.

        Args:
            joint_name: One of the six Arctos joint names.
            velocity: Unitless velocity in ``[-1.0, 1.0]``.
        """
        ...

    def multi_joint_jog(
        self,
        joint_names: Sequence[str],
        velocities: Sequence[float],
    ) -> None:
        """Publish velocity commands for multiple joints simultaneously."""
        ...

    def cartesian_jog(
        self,
        linear: Tuple[float, float, float],
        angular: Tuple[float, float, float],
    ) -> None:
        """Publish a cartesian twist command.

        Args:
            linear: ``(x, y, z)`` unitless velocities in ``[-1.0, 1.0]``.
            angular: ``(rx, ry, rz)`` unitless velocities in ``[-1.0, 1.0]``.
        """
        ...

    def stop(self) -> None:
        """Send zero-velocity commands on both joint and cartesian channels."""
        ...

    def get_joint_positions(self) -> Dict[str, float]:
        """Return the latest joint positions as ``{name: radians}``."""
        ...

    def get_ee_pose(self) -> Dict[str, float]:
        """Return the current end-effector pose.

        Returns:
            Dict with keys ``x, y, z`` (metres) and ``rx, ry, rz`` (radians).
        """
        ...
