"""Unit conversion between STM32 steps and ROS2 radians."""

import math
from typing import List


TWO_PI = 2.0 * math.pi


class UnitConverter:
    """Converts between steps (STM32) and radians (ROS2)."""

    def __init__(
        self,
        steps_per_rev: int,
        microsteps: int,
        gear_ratios: List[float],
        joint_inversions: List[bool],
    ):
        self._num_joints = len(gear_ratios)
        self._steps_per_revolution = [
            steps_per_rev * microsteps * gr for gr in gear_ratios
        ]
        self._joint_inversions = joint_inversions

    @property
    def num_joints(self) -> int:
        return self._num_joints

    def steps_to_rad(self, steps: float, joint_index: int) -> float:
        spr = self._steps_per_revolution[joint_index]
        rad = steps * (TWO_PI / spr)
        return -rad if self._joint_inversions[joint_index] else rad

    def steps_per_sec_to_rad_per_sec(self, sps: float, joint_index: int) -> float:
        spr = self._steps_per_revolution[joint_index]
        rad_s = sps * (TWO_PI / spr)
        return -rad_s if self._joint_inversions[joint_index] else rad_s

    def rad_to_steps(self, rad: float, joint_index: int) -> float:
        effective = -rad if self._joint_inversions[joint_index] else rad
        spr = self._steps_per_revolution[joint_index]
        return effective * (spr / TWO_PI)

    def rad_per_sec_to_steps_per_sec(self, rad_s: float, joint_index: int) -> float:
        effective = -rad_s if self._joint_inversions[joint_index] else rad_s
        spr = self._steps_per_revolution[joint_index]
        return effective * (spr / TWO_PI)
