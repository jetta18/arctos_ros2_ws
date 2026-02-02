"""Protocol definition for cartesian jog clients used by the CartesianJogWidget."""

from typing import Protocol


class CartesianJogClient(Protocol):
    """Abstraction for something that can execute cartesian jog commands.

    Keeps the Qt layer independent from MoveIt specifics.
    """

    def send_cartesian_step(self, axis: str, step_m: float, speed_scale: float) -> bool:
        """Send a cartesian step command.

        Args:
            axis: Axis to move ('x', 'y', 'z', 'rx', 'ry', 'rz')
            step_m: Step size in meters (for translation) or radians (for rotation)
            speed_scale: Speed scaling factor (0.0 to 1.0)

        Returns:
            True if command was accepted/executed successfully
        """
        ...

    def connect(self) -> bool:
        ...

    def disconnect(self) -> None:
        ...

    def is_connected(self) -> bool:
        ...

    def get_current_pose(self) -> dict:
        """Get current end-effector pose.

        Returns:
            dict with keys: 'x', 'y', 'z' (meters), 'rx', 'ry', 'rz' (radians)
        """
        ...
