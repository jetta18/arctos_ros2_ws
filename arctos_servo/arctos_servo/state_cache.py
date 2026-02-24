"""State cache helpers for arctos_servo."""

from __future__ import annotations

import threading
from typing import Dict, Iterable

import rclpy
from sensor_msgs.msg import JointState

from .math_utils import euler_from_quaternion


class JointStateCache:
    """Thread-safe joint state cache."""

    def __init__(self, joint_names: Iterable[str]) -> None:
        self._joint_names = list(joint_names)
        self._lock = threading.Lock()
        self._positions: Dict[str, float] = {
            name: 0.0 for name in self._joint_names
        }

    def update_from_msg(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        with self._lock:
            for name in self._joint_names:
                if name in name_to_pos:
                    self._positions[name] = float(name_to_pos[name])

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._positions)

    def get_position(self, joint_name: str) -> float:
        with self._lock:
            return self._positions.get(joint_name, 0.0)


def get_ee_pose(
    tf_buffer,
    base_frame: str,
    ee_frame: str,
    timeout_sec: float = 0.5,
) -> Dict[str, float]:
    """Return end-effector pose using TF buffer."""
    zero_pose = {
        'x': 0.0, 'y': 0.0, 'z': 0.0,
        'rx': 0.0, 'ry': 0.0, 'rz': 0.0,
    }
    if tf_buffer is None:
        return zero_pose

    try:
        tf = tf_buffer.lookup_transform(
            base_frame,
            ee_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=timeout_sec),
        )
        pos = tf.transform.translation
        rot = tf.transform.rotation
        rpy = euler_from_quaternion(rot.x, rot.y, rot.z, rot.w)
        return {
            'x': float(pos.x), 'y': float(pos.y), 'z': float(pos.z),
            'rx': float(rpy[0]), 'ry': float(rpy[1]), 'rz': float(rpy[2]),
        }
    except Exception:
        return zero_pose
