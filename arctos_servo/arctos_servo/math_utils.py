"""Math helpers for arctos_servo."""

from __future__ import annotations

import math
from typing import Tuple


def euler_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float]:
    """Convert quaternion to (roll, pitch, yaw) in radians."""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return (roll, pitch, yaw)
