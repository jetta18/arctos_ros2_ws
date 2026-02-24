"""Shared runtime application state for Arctos GUI.

This module holds mutable state that is shared across components at runtime.
It is intentionally kept separate from persisted settings (SettingsManager).

No PyQt5 or rclpy imports are allowed here.
"""

from __future__ import annotations

import threading
from typing import Any

AXIS_NAMES: tuple[str, ...] = ("X", "Y", "Z", "A", "B", "C")


class AppState:
    """Container for runtime state shared across GUI components.

    Thread-safe via an internal lock. Components should read state via
    properties and update it via the provided setters.

    Attributes:
        endstop_states: Mapping of axis name to triggered state (True = triggered).
        robot_connected: Whether the robot hardware is currently connected.
        robot_state: Arbitrary key/value pairs sent by the robot (extensible).
    """

    def __init__(self) -> None:
        """Initializes AppState with safe default values."""
        self._lock = threading.Lock()
        self._endstop_states: dict[str, bool] = {axis: False for axis in AXIS_NAMES}
        self._robot_connected: bool = False
        self._robot_state: dict[str, Any] = {}

    @property
    def endstop_states(self) -> dict[str, bool]:
        """Returns a snapshot of the current endstop states."""
        with self._lock:
            return dict(self._endstop_states)

    def set_endstop_state(self, axis: str, triggered: bool) -> None:
        """Updates the triggered state for a single endstop.

        Args:
            axis: Axis name, one of X/Y/Z/A/B/C.
            triggered: True if the endstop is triggered.
        """
        with self._lock:
            self._endstop_states[axis] = triggered

    def set_endstop_states(self, states: dict[str, bool]) -> None:
        """Replaces all endstop states at once.

        Args:
            states: Mapping of axis name to triggered state.
        """
        with self._lock:
            self._endstop_states.update(states)

    @property
    def robot_connected(self) -> bool:
        """Returns whether the robot hardware is connected."""
        with self._lock:
            return self._robot_connected

    @robot_connected.setter
    def robot_connected(self, value: bool) -> None:
        """Sets the robot connection state.

        Args:
            value: True if connected.
        """
        with self._lock:
            self._robot_connected = value

    @property
    def robot_state(self) -> dict[str, Any]:
        """Returns a snapshot of the current robot state dict."""
        with self._lock:
            return dict(self._robot_state)

    def update_robot_state(self, updates: dict[str, Any]) -> None:
        """Merges *updates* into the robot state dict.

        Args:
            updates: Key/value pairs to merge.
        """
        with self._lock:
            self._robot_state.update(updates)
