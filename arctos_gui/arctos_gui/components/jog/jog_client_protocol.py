"""Protocol definition for jog clients used by the JogWidget."""

from typing import Protocol


class JogClient(Protocol):
    """Abstraction for something that can execute jog commands.

    Keeps the Qt layer independent from ROS specifics.
    """

    def get_current_position(self, axis_index: int) -> float:
        """Return the last known joint position for *axis_index* in radians."""

        ...

    def send_jog(self, axis_index: int, delta_rad: float, velocity_rad_s: float) -> None:
        """Send a jog command for the given axis and delta at the requested velocity."""

        ...

    def connect(self) -> bool:
        """Establish any backend connection required by the jog client."""

        ...

    def disconnect(self) -> None:
        """Tear down the backend connection if one is active."""

        ...

    def is_connected(self) -> bool:
        """Return True when the backend transport is ready to accept jog commands."""

        ...
