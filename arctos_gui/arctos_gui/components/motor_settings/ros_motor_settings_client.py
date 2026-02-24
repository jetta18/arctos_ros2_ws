"""CAN bus client for MKS Servo 42D/57D motor settings.

Uses the mks_servo_can library (MksServo) to send individual CAN commands.
The new API is synchronous — one MksServo instance per motor axis, sharing
a single python-can Bus and Notifier.

The ``slcan`` interface is used exclusively (CANable / CANable2 on Linux).
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Optional

import can
from mks_servo_can import MksServo
from mks_servo_can.mks_enums import (
    CanBitrate,
    Direction,
    Enable,
    EndStopLevel,
    HoldingStrength,
    WorkMode,
)

logger = logging.getLogger(__name__)

# Axis name → default CAN ID mapping
_DEFAULT_CAN_IDS: dict[str, int] = {
    "X": 1,
    "Y": 2,
    "Z": 3,
    "A": 4,
    "B": 5,
    "C": 6,
}

# GUI label → MksServo enum mappings
_HOLD_PCT_ENUMS: dict[str, HoldingStrength] = {
    f"{(i + 1) * 10}%": list(HoldingStrength)[i] for i in range(9)
}

_WORK_MODE_ENUMS: dict[str, WorkMode] = {
    "CR_OPEN": WorkMode.CrOpen,
    "CR_CLOSE": WorkMode.CrClose,
    "CR_vFOC": WorkMode.CrvFoc,
    "SR_OPEN": WorkMode.SrOpen,
    "SR_CLOSE": WorkMode.SrClose,
    "SR_vFOC": WorkMode.SrvFoc,
}

_DIR_ENUMS: dict[str, Direction] = {
    "CW": Direction.CW,
    "CCW": Direction.CCW,
}

_ENDSTOP_LEVEL_ENUMS: dict[str, EndStopLevel] = {
    "Low": EndStopLevel.Low,
    "High": EndStopLevel.High,
}

_CAN_BITRATE_ENUMS: dict[str, CanBitrate] = {
    "125K": CanBitrate.Rate125K,
    "250K": CanBitrate.Rate250K,
    "500K": CanBitrate.Rate500K,
    "1M": CanBitrate.Rate1M,
}

_HOMING_SPEED_VALUES: dict[str, int] = {
    str(v): v for v in [30, 60, 90, 120, 150, 180, 300, 600, 1200, 3000]
}


class ArctosMotorSettingsClient:
    """Sends individual MKS Servo CAN commands via the mks_servo_can library.

    All public methods are synchronous.  CAN send/receive uses internal
    timeouts from the MksServo class (default 1 s).
    """

    def __init__(self, can_ids: Optional[dict[str, int]] = None) -> None:
        """Initialize the instance."""
        self._can_ids: dict[str, int] = can_ids or dict(_DEFAULT_CAN_IDS)
        self._bus: Optional[can.Bus] = None
        self._notifier: Optional[can.Notifier] = None
        self._servos: dict[str, MksServo] = {}
        self._connected = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self, interface_type: str, channel: str, bitrate: int) -> bool:
        """Opens the CAN bus and creates per-axis MksServo instances.

        Args:
            interface_type: Ignored — always uses ``"slcan"``.
            channel: Serial device path, e.g. ``"/dev/ttyACM0"``.
            bitrate: CAN bus bitrate in bps.

        Returns:
            True on success, False on failure.
        """
        with self._lock:
            try:
                self._bus = can.interface.Bus(
                    interface="slcan",
                    channel=channel,
                    bitrate=bitrate,
                )
                self._notifier = can.Notifier(self._bus, [])
                self._servos = {
                    axis: MksServo(self._bus, self._notifier, cid)
                    for axis, cid in self._can_ids.items()
                }
                self._connected = True
                logger.info("CAN connected: slcan %s @ %d bps", channel, bitrate)
                return True
            except Exception:
                logger.exception("CAN connect failed")
                self._cleanup()
                return False

    def disconnect(self) -> None:
        """Closes the CAN bus and cleans up."""
        with self._lock:
            self._cleanup()
            logger.info("CAN disconnected")

    def _cleanup(self) -> None:
        """Release ROS resources and reset local state."""
        self._connected = False
        self._servos.clear()
        if self._notifier is not None:
            try:
                self._notifier.stop()
            except Exception:
                logger.debug("Notifier stop error", exc_info=True)
            self._notifier = None
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                logger.debug("Bus shutdown error", exc_info=True)
            self._bus = None

    def is_connected(self) -> bool:
        """Return True when the backend connection is established."""
        return self._connected

    def update_can_id(self, axis: str, can_id: int) -> None:
        """Updates the CAN ID for *axis* and recreates its MksServo instance.

        Args:
            axis: Axis name (X/Y/Z/A/B/C).
            can_id: New CAN ID (1–2047).
        """
        self._can_ids[axis] = can_id
        if self._connected and self._bus is not None and self._notifier is not None:
            self._servos[axis] = MksServo(self._bus, self._notifier, can_id)

    # ------------------------------------------------------------------
    # Send command dispatcher
    # ------------------------------------------------------------------

    def send_command(self, axis_name: str, command: str, value: Any) -> bool:
        """Sends a single CAN parameter command to the motor.

        Args:
            axis_name: Axis name (X/Y/Z/A/B/C).
            command: Command identifier string.
            value: Value to send.

        Returns:
            True if the motor acknowledged success, False otherwise.
        """
        if not self._connected:
            logger.warning("send_command called while not connected")
            return False

        servo = self._servos.get(axis_name)
        if servo is None:
            logger.warning("No servo for axis %s", axis_name)
            return False

        try:
            handler = _COMMAND_DISPATCH.get(command)
            if handler is None:
                logger.warning("Unknown command: %s", command)
                return False
            result = handler(servo, value)
            logger.info("Axis %s: %s=%r → %s", axis_name, command, value, result)
            return result is not None
        except Exception:
            logger.exception(
                "send_command %s=%r to axis %s failed", command, value, axis_name
            )
            return False

    # ------------------------------------------------------------------
    # Read motor status
    # ------------------------------------------------------------------

    def read_motor_status(self, axis_name: str) -> dict[str, Any]:
        """Reads live status from the motor for the given axis.

        Args:
            axis_name: Axis name (X/Y/Z/A/B/C).

        Returns:
            Dict with speed_rpm, io_status, encoder, protection_state.
        """
        if not self._connected:
            return {"error": "Not connected"}

        servo = self._servos.get(axis_name)
        if servo is None:
            return {"error": f"No servo for axis {axis_name}"}

        result: dict[str, Any] = {}

        for key, reader in _STATUS_READERS.items():
            try:
                result[key] = reader(servo)
            except Exception as exc:
                result[key] = f"Error: {exc}"

        return result


# ------------------------------------------------------------------
# Command dispatch table (module-level, stateless)
# ------------------------------------------------------------------

def _send_work_mode(servo: MksServo, v: Any) -> Any:
    """Send the work mode."""
    return servo.set_work_mode(_WORK_MODE_ENUMS.get(str(v), WorkMode.SrvFoc))


def _send_working_current(servo: MksServo, v: Any) -> Any:
    """Send the working current."""
    return servo.set_working_current(int(v))


def _send_holding_current(servo: MksServo, v: Any) -> Any:
    """Send the holding current."""
    return servo.set_holding_current(
        _HOLD_PCT_ENUMS.get(str(v), HoldingStrength.FIFTHTY_PERCENT)
    )


def _send_subdivisions(servo: MksServo, v: Any) -> Any:
    """Send the subdivisions."""
    return servo.set_subdivisions(int(v))


def _send_run_direction(servo: MksServo, v: Any) -> Any:
    """Send the run direction."""
    return servo.set_motor_rotation_direction(
        _DIR_ENUMS.get(str(v), Direction.CW)
    )


def _send_subdiv_interpolation(servo: MksServo, v: Any) -> Any:
    """Send the subdiv interpolation."""
    enable = Enable.Enable if v else Enable.Disable
    return servo.set_subdivision_interpolation(enable)


def _send_stall_protection(servo: MksServo, v: Any) -> Any:
    """Send the stall protection."""
    enable = Enable.Enable if v else Enable.Disable
    return servo.set_motor_shaft_locked_rotor_protection(enable)


def _send_can_id(servo: MksServo, v: Any) -> Any:
    """Send the can id."""
    return servo.set_can_id(int(v))


def _send_can_bitrate(servo: MksServo, v: Any) -> Any:
    """Send the can bitrate."""
    return servo.set_can_bitrate(
        _CAN_BITRATE_ENUMS.get(str(v), CanBitrate.Rate500K)
    )


def _send_set_home(servo: MksServo, v: Any) -> Any:
    """Sends the full set_home command.

    *v* is expected to be a dict with keys: home_trig, home_dir,
    home_speed, end_limit.  Missing keys use safe defaults.
    """
    if not isinstance(v, dict):
        v = {}
    return servo.set_home(
        homeTrig=_ENDSTOP_LEVEL_ENUMS.get(str(v.get("home_trig", "Low")), EndStopLevel.Low),
        homeDir=_DIR_ENUMS.get(str(v.get("home_dir", "CW")), Direction.CW),
        homeSpeed=int(v.get("home_speed", 60)),
        endLimit=Enable.Enable if v.get("end_limit") else Enable.Disable,
    )


_COMMAND_DISPATCH: dict[str, Any] = {
    "work_mode": _send_work_mode,
    "working_current": _send_working_current,
    "hold_current_pct": _send_holding_current,
    "microsteps": _send_subdivisions,
    "run_direction": _send_run_direction,
    "subdiv_interpolation": _send_subdiv_interpolation,
    "stall_protection": _send_stall_protection,
    "can_id": _send_can_id,
    "can_bitrate": _send_can_bitrate,
    "set_home": _send_set_home,
}

_STATUS_READERS: dict[str, Any] = {
    "speed_rpm": lambda s: s.read_motor_speed(),
    "encoder": lambda s: s.read_encoder_value_addition(),
    "io_status": lambda s: s.read_io_port_status(),
    "protection": lambda s: s.read_motor_shaft_protection_state(),
    "motor_status": lambda s: s.query_motor_status(),
}


