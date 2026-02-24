"""Protocol interface for the robot status client.

Defines the contract that any robot status backend must satisfy.
No ROS or PyQt5 imports are allowed here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from ...backend.app_state import AXIS_NAMES


@dataclass
class EndstopAxisData:
    """Endstop state for a single axis (both directions).

    Attributes:
        min_triggered: True if the MIN endstop is currently triggered.
        max_triggered: True if the MAX endstop is currently triggered.
        min_trigger_count: Cumulative MIN trigger count from the STM32.
        max_trigger_count: Cumulative MAX trigger count from the STM32.
    """

    min_triggered: bool = False
    max_triggered: bool = False
    min_trigger_count: int = 0
    max_trigger_count: int = 0


@dataclass
class JointStateData:
    """State for a single joint/axis.
    
    Attributes:
        position_rad: Joint position in radians.
        velocity_rad_s: Joint velocity in rad/s.
        position_steps: Joint position in raw motor steps.
        is_homed: True if the axis has been successfully homed.
    """
    
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    position_steps: float = 0.0
    is_homed: bool = False


@dataclass
class DiagnosticData:
    """Comprehensive system state and diagnostics.
    
    Attributes:
        system_state: String representation of the current system state.
        uptime_s: STM32 uptime in seconds.
        servo_pulse_us: Current servo pulse width in microseconds.
        broadcast_seq: UDP broadcast sequence number.
        homing_active: True if homing sequence is currently running.
        trajectory_id: ID of the currently loaded/running trajectory.
        traj_points_loaded: Number of trajectory points loaded.
        traj_current_segment: Current trajectory segment being executed.
        traj_total_segments: Total segments in the current trajectory.
    """
    
    system_state: str = "UNKNOWN"
    uptime_s: float = 0.0
    servo_pulse_us: int = 0
    broadcast_seq: int = 0
    homing_active: bool = False
    trajectory_id: int = 0
    traj_points_loaded: int = 0
    traj_current_segment: int = 0
    traj_total_segments: int = 0


STM32_DISCONNECTED = 0
STM32_CONNECTED = 1
STM32_RECONNECTING = 2


@dataclass
class ConnectionData:
    """STM32 connection status.

    Attributes:
        status: One of STM32_DISCONNECTED, STM32_CONNECTED, STM32_RECONNECTING.
        last_received_time: Epoch timestamp of last received broadcast.
        reconnect_attempts: Cumulative reconnection attempts.
        message: Human-readable status message.
    """

    status: int = STM32_DISCONNECTED
    last_received_time: float = 0.0
    reconnect_attempts: int = 0
    message: str = "No data"


def default_endstop_data() -> dict[str, EndstopAxisData]:
    """Returns a default endstop data dict with all axes cleared.

    Returns:
        Mapping of axis name to cleared EndstopAxisData.
    """
    return {axis: EndstopAxisData() for axis in AXIS_NAMES}


def default_joint_data() -> dict[str, JointStateData]:
    """Returns a default joint data dict with all axes cleared.

    Returns:
        Mapping of axis name to cleared JointStateData.
    """
    return {axis: JointStateData() for axis in AXIS_NAMES}


class RobotStatusClient(Protocol):
    """Protocol for reading robot status data.

    Implementations may connect to ROS topics, simulate data, or read
    from any other source. The widget depends only on this interface.
    """

    def get_endstop_data(self) -> dict[str, EndstopAxisData]:
        """Returns the current per-axis endstop states and trigger counts.

        Returns:
            Mapping of axis name (X/Y/Z/A/B/C) to EndstopAxisData.
        """
        ...

    def get_joint_data(self) -> dict[str, JointStateData]:
        """Returns the current per-axis joint states (positions, velocity, homed).

        Returns:
            Mapping of axis name (X/Y/Z/A/B/C) to JointStateData.
        """
        ...

    def get_diagnostic_data(self) -> DiagnosticData:
        """Returns system state and diagnostic information.

        Returns:
            DiagnosticData object containing system state and diagnostics.
        """
        ...

    def get_connection_data(self) -> ConnectionData:
        """Returns the current STM32 connection status.

        Returns:
            ConnectionData with status, timestamps, and attempt count.
        """
        ...

    def request_reconnect(self) -> tuple[bool, str]:
        """Trigger an immediate reconnection attempt.

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
