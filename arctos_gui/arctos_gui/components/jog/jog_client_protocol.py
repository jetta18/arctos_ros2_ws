"""Protocol definition for jog clients used by the JogWidget."""

from typing import Protocol


class JogClient(Protocol):
    """Abstraction for something that can execute jog commands.

    Keeps the Qt layer independent from ROS specifics.
    """

    def send_jog(self, axis_index: int, delta_rad: float, velocity_rad_s: float) -> None:
        ...

    def connect(self) -> bool:
        ...

    def disconnect(self) -> None:
        ...

    def is_connected(self) -> bool:
        ...
