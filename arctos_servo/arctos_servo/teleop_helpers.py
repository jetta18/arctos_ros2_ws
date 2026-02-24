"""Shared keyboard teleop helpers for arctos_servo."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from .constants import JOINT_NAMES
from .servo_client import ServoClient

KEY_POLL_TIMEOUT_S = 0.01
KEY_HOLD_DURATION_S = 0.5

INSTRUCTIONS_JOINT = """
╔══════════════════════════════════════════╗
║   Arctos Servo Teleop  ·  JOINT MODE    ║
╠══════════════════════════════════════════╣
║  1-6  = Select joint                     ║
║    1 = X_joint   4 = A_joint             ║
║    2 = Y_joint   5 = B_joint             ║
║    3 = Z_joint   6 = C_joint             ║
║                                          ║
║  W / S  = Jog selected joint  + / −      ║
║                                          ║
║  Tab = cartesian mode                    ║
║  Space = stop  |  Esc = quit             ║
╚══════════════════════════════════════════╝"""

INSTRUCTIONS_CARTESIAN = """
╔══════════════════════════════════════════╗
║  Arctos Servo Teleop · CARTESIAN MODE   ║
╠══════════════════════════════════════════╣
║  W / S  = +X / −X                        ║
║  A / D  = +Y / −Y                        ║
║  Q / E  = +Z / −Z                        ║
║  I / K  = +RX / −RX                      ║
║  J / L  = +RY / −RY                      ║
║  U / O  = +RZ / −RZ                      ║
║                                          ║
║  Tab = joint mode                        ║
║  Space = stop  |  Esc = quit             ║
╚══════════════════════════════════════════╝"""

_CARTESIAN_KEY_MAP = {
    'w': (1, 0, 0, 0, 0, 0),
    's': (-1, 0, 0, 0, 0, 0),
    'a': (0, 1, 0, 0, 0, 0),
    'd': (0, -1, 0, 0, 0, 0),
    'q': (0, 0, 1, 0, 0, 0),
    'e': (0, 0, -1, 0, 0, 0),
    'i': (0, 0, 0, 1, 0, 0),
    'k': (0, 0, 0, -1, 0, 0),
    'j': (0, 0, 0, 0, 1, 0),
    'l': (0, 0, 0, 0, -1, 0),
    'u': (0, 0, 0, 0, 0, 1),
    'o': (0, 0, 0, 0, 0, -1),
}


@dataclass
class TeleopState:
    """Mutable state for the teleop loop."""

    cartesian_mode: bool = False
    selected_joint: int = 0
    velocity: float = 0.0
    cartesian_cmd: tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )
    last_key_time: float = 0.0
    running: bool = True


def process_key(key: Optional[str], state: TeleopState) -> tuple[Optional[str], bool]:
    """Update state from a single key press.

    Returns:
        (mode_label, joint_changed)
    """
    if key is None:
        return None, False

    if key == '\x1b':
        state.running = False
        return None, False

    if key == '\t':
        state.cartesian_mode = not state.cartesian_mode
        state.velocity = 0.0
        state.cartesian_cmd = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        state.last_key_time = 0.0
        return 'CARTESIAN' if state.cartesian_mode else 'JOINT', False

    if key == ' ':
        state.velocity = 0.0
        state.cartesian_cmd = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        state.last_key_time = 0.0
        return None, False

    lower = key.lower()

    if state.cartesian_mode:
        if lower in _CARTESIAN_KEY_MAP:
            cmd = _CARTESIAN_KEY_MAP[lower]
            state.cartesian_cmd = tuple(float(v) for v in cmd)
            state.last_key_time = time.monotonic()
    else:
        if lower in '123456':
            state.selected_joint = int(lower) - 1
            return None, True
        elif lower == 'w':
            state.velocity = 1.0
            state.last_key_time = time.monotonic()
        elif lower == 's':
            state.velocity = -1.0
            state.last_key_time = time.monotonic()

    return None, False


def send_commands(client: ServoClient, state: TeleopState) -> None:
    """Publish the current command based on state."""
    elapsed = (
        time.monotonic() - state.last_key_time
        if state.last_key_time > 0 else float('inf')
    )
    timed_out = elapsed > KEY_HOLD_DURATION_S

    if state.cartesian_mode:
        if timed_out:
            state.cartesian_cmd = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        lx, ly, lz, ax, ay, az = state.cartesian_cmd
        client.cartesian_jog(
            linear=(lx, ly, lz),
            angular=(ax, ay, az),
        )
    else:
        if timed_out:
            state.velocity = 0.0
        client.joint_jog_by_index(state.selected_joint, state.velocity)


def current_joint_label(state: TeleopState) -> str:
    """Return the currently selected joint label."""
    return JOINT_NAMES[state.selected_joint]
