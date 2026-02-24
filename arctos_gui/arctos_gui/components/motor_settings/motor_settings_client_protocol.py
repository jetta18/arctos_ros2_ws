"""Protocol interface for the motor settings CAN client.

Defines the contract that any motor settings backend must satisfy.
No ROS or PyQt5 imports are allowed here.
"""

from __future__ import annotations

from typing import Any, Protocol


class MotorSettingsClient(Protocol):
    """Protocol for reading and writing MKS Servo motor parameters via CAN bus."""

    def connect(self, interface_type: str, channel: str, bitrate: int) -> bool:
        """Opens the CAN interface connection.

        Args:
            interface_type: python-can interface type (e.g. ``"slcan"``).
            channel: Interface channel (e.g. ``"/dev/ttyACM0"``).
            bitrate: CAN bus bitrate in bps (e.g. 500000).

        Returns:
            True on success, False on failure.
        """
        ...

    def disconnect(self) -> None:
        """Closes the CAN interface connection."""
        ...

    def is_connected(self) -> bool:
        """Returns True when the CAN interface is open.

        Returns:
            True if connected.
        """
        ...

    def send_command(self, axis_name: str, command: str, value: Any) -> bool:
        """Sends a single CAN parameter command to the motor for the given axis.

        Args:
            axis_name: Axis name (X/Y/Z/A/B/C).
            command: Command identifier string (e.g. ``"working_current"``).
            value: Value to send (type depends on command).

        Returns:
            True if the motor acknowledged success, False otherwise.
        """
        ...

    def read_motor_status(self, axis_name: str) -> dict[str, Any]:
        """Reads live status from the motor for the given axis.

        Args:
            axis_name: Axis name (X/Y/Z/A/B/C).

        Returns:
            Dict with keys such as ``"speed_rpm"``, ``"io_status"``,
            ``"encoder_value"``, ``"protection_state"``.
        """
        ...
