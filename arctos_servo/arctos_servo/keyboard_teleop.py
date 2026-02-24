"""Keyboard teleop (test utility) for Arctos MoveIt Servo.

Uses the ``ServoClient`` API. Press **Tab** to toggle between joint
jog and cartesian jog mode.
"""

import select
import sys
import termios
import time
import tty
from typing import Optional

from .constants import PUBLISH_RATE_HZ
from .servo_client import ServoClient
from .teleop_helpers import (
    INSTRUCTIONS_CARTESIAN,
    INSTRUCTIONS_JOINT,
    KEY_POLL_TIMEOUT_S,
    TeleopState,
    current_joint_label,
    process_key,
    send_commands,
)


def _read_key() -> Optional[str]:
    """Return a key if one is available, else None."""
    if select.select([sys.stdin], [], [], KEY_POLL_TIMEOUT_S)[0]:
        return sys.stdin.read(1)
    return None


def main(args=None) -> None:
    """Entry point for the keyboard teleop CLI tool."""
    if not sys.stdin.isatty():
        print(
            'ERROR: keyboard_teleop requires a terminal (TTY).\n'
            'Run directly:  ros2 run arctos_servo keyboard_teleop',
            file=sys.stderr,
        )
        sys.exit(1)

    original_settings = termios.tcgetattr(sys.stdin)

    client = ServoClient(node_name='keyboard_teleop')
    print('Connecting to MoveIt Servo...')

    if not client.connect():
        print('ERROR: Failed to connect to MoveIt Servo.', file=sys.stderr)
        sys.exit(1)

    state = TeleopState()
    last_joint_label = current_joint_label(state)
    period = 1.0 / PUBLISH_RATE_HZ

    print(INSTRUCTIONS_JOINT)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while state.running:
            t0 = time.monotonic()
            mode, joint_changed = process_key(_read_key(), state)
            if mode:
                print(f'\n--- Switched to {mode} mode ---')
                print(
                    INSTRUCTIONS_CARTESIAN
                    if state.cartesian_mode
                    else INSTRUCTIONS_JOINT
                )
                if not state.cartesian_mode:
                    last_joint_label = current_joint_label(state)
                    print(f'  Joint: {last_joint_label}')
            if not state.cartesian_mode and joint_changed:
                current_label = current_joint_label(state)
                if current_label != last_joint_label:
                    last_joint_label = current_label
                    print(f'  Joint: {current_label}')
            if state.running:
                send_commands(client, state)
            remaining = period - (time.monotonic() - t0)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        client.stop()
        client.disconnect()
        print('\nDisconnected.')


if __name__ == '__main__':
    main()
