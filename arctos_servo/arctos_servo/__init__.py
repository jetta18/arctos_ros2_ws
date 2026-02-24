"""arctos_servo — MoveIt Servo API for the Arctos robot.

Quick start::

    from arctos_servo import ServoClient

    client = ServoClient()
    client.connect()
    client.joint_jog('X_joint', 0.5)
    client.disconnect()
"""

from .constants import JOINT_NAMES, NUM_JOINTS  # noqa: F401
from .servo_client import ServoClient  # noqa: F401

__all__ = ['ServoClient', 'JOINT_NAMES', 'NUM_JOINTS']
