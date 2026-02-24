"""Protocol interface for homing clients used by the HomingWidget.

Defines the contract that any homing backend must satisfy.
No ROS or PyQt5 imports are allowed here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from ...backend.app_state import AXIS_NAMES

NUM_AXES = len(AXIS_NAMES)


@dataclass
class HomingAxisStatus:
    """Live homing status for a single axis.

    Attributes:
        state: Current homing state name (IDLE, SEEKING, RETURNING, COMPLETE, ERROR).
        active: True if this axis is currently homing.
        is_homed: True if this axis has been successfully homed to position.
    """

    state: str = "IDLE"
    active: bool = False
    is_homed: bool = False


@dataclass
class HomingStatus:
    """Aggregated homing status for all axes.

    Attributes:
        any_active: True if any axis is currently homing.
        active_axis: Index of the currently homing axis (-1 if none).
        active_state: State name of the currently homing axis.
        axes: Per-axis status list.
        is_homed_bitmask: Raw bitmask from STM32 (bit i = axis i homed).
    """

    any_active: bool = False
    active_axis: int = -1
    active_state: str = "IDLE"
    axes: list[HomingAxisStatus] = field(default_factory=lambda: [
        HomingAxisStatus() for _ in range(NUM_AXES)
    ])
    is_homed_bitmask: int = 0


def default_homing_status() -> HomingStatus:
    """Returns a default HomingStatus with all axes idle."""
    return HomingStatus()


class HomingClient(Protocol):
    """Abstraction for homing operations.

    Keeps the Qt layer independent from ROS specifics.
    """

    def home_axis_to_endstop(
        self, axis: int, direction: int, velocity_rad_s: float,
    ) -> tuple[bool, str]:
        """Start homing a single axis to its endstop.

        Args:
            axis: Axis index 0-5.
            direction: 0=MIN, 1=MAX.
            velocity_rad_s: Homing velocity in rad/s.

        Returns:
            (success, message) tuple.
        """
        ...

    def stop_homing(self) -> tuple[bool, str]:
        """Stop all active homing operations.

        Returns:
            (success, message) tuple.
        """
        ...

    def home_axis_to_position(
        self, axis: int, velocity_rad_s: float, offset_steps: int,
    ) -> tuple[bool, str]:
        """Home axis to endstop then move to offset position.

        Args:
            axis: Axis index 0-5.
            velocity_rad_s: Homing velocity in rad/s (0 = use default).
            offset_steps: Signed offset from endstop to home position.

        Returns:
            (success, message) tuple.
        """
        ...

    def get_homing_status(self) -> HomingStatus:
        """Returns the current homing status from the state broadcast.

        Returns:
            Aggregated homing status.
        """
        ...

    def get_current_position(self, axis: int) -> float:
        """Returns the current position of an axis in radians.

        Args:
            axis: Axis index 0-5.

        Returns:
            Position in radians.
        """
        ...

    def get_current_position_steps(self, axis: int) -> float:
        """Returns the current position of an axis in steps.

        Args:
            axis: Axis index 0-5.

        Returns:
            Position in steps.
        """
        ...

    def jog_axis(
        self, axis: int, distance_steps: int, velocity_rad_s: float,
    ) -> tuple[bool, str]:
        """Jog an axis by a relative distance in steps.

        Args:
            axis: Axis index 0-5.
            distance_steps: Signed distance in steps.
            velocity_rad_s: Jog velocity in rad/s.

        Returns:
            (success, message) tuple.
        """
        ...

    def set_axis_position(self, axis: int, position_steps: float) -> tuple[bool, str]:
        """Set the step counter for a single axis to an arbitrary value.

        Args:
            axis: Axis index 0-5.
            position_steps: New position value in steps (use 0.0 to reset).

        Returns:
            (success, message) tuple.
        """
        ...

    def set_home_offset(
        self, axis: int, direction: int, offset_steps: int,
    ) -> tuple[bool, str]:
        """Set the home offset and direction for an axis.

        Args:
            axis: Axis index 0-5.
            direction: Endstop direction (0=MIN, 1=MAX).
            offset_steps: Signed offset from endstop to home position.

        Returns:
            (success, message) tuple.
        """
        ...

    def is_connected(self) -> bool:
        """Returns True if the client has an active data source.

        Returns:
            True when connected and receiving data.
        """
        ...
