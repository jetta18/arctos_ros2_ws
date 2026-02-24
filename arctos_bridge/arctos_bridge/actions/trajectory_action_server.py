"""FollowJointTrajectory action server for the Arctos bridge.

Uploads trajectory points to the STM32 via UDP, executes, and monitors
completion via the state broadcast stream.
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

from arctos_bridge.protocol.stm32_protocol import STM32CommandClient, NUM_AXES
from arctos_bridge.protocol.unit_conversion import UnitConverter

MAX_TRAJ_POINTS = 128

# System state constants (must match arctos_system_state_t)
SYS_IDLE = 0
SYS_TRAJ_RUNNING = 3
SYS_TRAJ_COMPLETE = 4
SYS_ERROR = 5


class TrajectoryActionServer:
    """Manages the FollowJointTrajectory action lifecycle."""

    def __init__(
        self,
        node: Node,
        cmd_client: STM32CommandClient,
        converter: UnitConverter,
        joint_names: list,
        get_system_state_fn,
    ):
        self._node = node
        self._cmd_client = cmd_client
        self._converter = converter
        self._joint_names = joint_names
        self._get_system_state = get_system_state_fn
        self._next_traj_id = 1
        self._lock = threading.Lock()
        self._active_goal = None
        self._traj_running_seen = False

        self._action_server = ActionServer(
            node,
            FollowJointTrajectory,
            "/arctos_controller/follow_joint_trajectory",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

    def _goal_callback(self, goal_request):
        goal_names = goal_request.trajectory.joint_names
        if len(goal_names) != len(self._joint_names):
            self._node.get_logger().error(
                f"Goal joints size {len(goal_names)} != "
                f"expected {len(self._joint_names)}"
            )
            return GoalResponse.REJECT

        for name in self._joint_names:
            if name not in goal_names:
                self._node.get_logger().error(
                    f"Goal missing required joint '{name}'"
                )
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self._node.get_logger().info("Trajectory cancel requested")
        try:
            self._cmd_client.trajectory_cancel()
        except Exception as e:
            self._node.get_logger().error(f"Cancel command failed: {e}")
        return CancelResponse.ACCEPT

    def _set_trajectory_active(self, active: bool):
        with self._node._trajectory_active_lock:
            self._node._trajectory_active = active

    def _execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory

        with self._lock:
            traj_id = self._next_traj_id
            self._next_traj_id += 1

        goal_names = list(traj.joint_names)
        mapping = [goal_names.index(n) for n in self._joint_names]

        last_pt = traj.points[-1]
        planned_duration = (
            last_pt.time_from_start.sec
            + last_pt.time_from_start.nanosec / 1e9
        )
        self._node.get_logger().info(
            f"Uploading trajectory id={traj_id}, "
            f"{len(traj.points)} points, "
            f"duration={planned_duration:.3f}s"
        )

        result = FollowJointTrajectory.Result()

        self._set_trajectory_active(True)
        try:
            if not self._upload_trajectory(traj, traj_id, mapping):
                self._node.get_logger().error("Trajectory upload failed")
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                goal_handle.abort()
                return result

            result = self._monitor_execution(goal_handle, planned_duration)
            return result
        finally:
            self._set_trajectory_active(False)

    def _upload_trajectory(
        self,
        traj: JointTrajectory,
        traj_id: int,
        mapping: list,
    ) -> bool:
        num_points = min(len(traj.points), MAX_TRAJ_POINTS)
        if len(traj.points) > MAX_TRAJ_POINTS:
            self._node.get_logger().warn(
                f"Trajectory has {len(traj.points)} points, "
                f"capping at {MAX_TRAJ_POINTS}"
            )

        points = []
        for i in range(num_points):
            pt = traj.points[i]
            time_ms = int(
                pt.time_from_start.sec * 1000
                + pt.time_from_start.nanosec // 1_000_000
            )

            positions_steps = [0.0] * NUM_AXES
            velocities_steps = [0.0] * NUM_AXES

            for j in range(len(self._joint_names)):
                gi = mapping[j]
                pos_rad = pt.positions[gi] if gi < len(pt.positions) else 0.0
                vel_rps = pt.velocities[gi] if gi < len(pt.velocities) else 0.0
                positions_steps[j] = self._converter.rad_to_steps(pos_rad, j)
                velocities_steps[j] = self._converter.rad_per_sec_to_steps_per_sec(
                    vel_rps, j
                )

            points.append((i, time_ms, positions_steps, velocities_steps))

            if i == 0 or i == num_points - 1:
                self._node.get_logger().info(
                    f"  pt[{i}] t={time_ms}ms "
                    f"pos_steps={[f'{p:.1f}' for p in positions_steps]} "
                    f"vel_steps={[f'{v:.1f}' for v in velocities_steps]}"
                )

        try:
            success, msg = self._cmd_client.trajectory_upload_and_execute(
                traj_id, num_points, points
            )
            if not success:
                self._node.get_logger().error(
                    f"Trajectory upload/execute failed: {msg}"
                )
                return False
        except Exception as e:
            self._node.get_logger().error(
                f"Trajectory upload/execute exception: {e}"
            )
            return False

        self._node.get_logger().info(
            f"Trajectory {traj_id} uploaded and executing ({num_points} points)"
        )
        return True

    def _monitor_execution(self, goal_handle, planned_duration: float):
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(self._joint_names)

        traj_running_seen = False
        timeout = planned_duration + 5.0
        start_time = time.monotonic()
        last_keepalive = start_time

        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - start_time

            if goal_handle.is_cancel_requested:
                self._node.get_logger().info("Trajectory cancelled by client")
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                goal_handle.canceled()
                return result

            sys_state = self._get_system_state()

            if sys_state == SYS_TRAJ_RUNNING:
                traj_running_seen = True

            completed = (
                sys_state == SYS_TRAJ_COMPLETE
                or (traj_running_seen and sys_state == SYS_IDLE)
            )

            if completed:
                self._node.get_logger().info(
                    f"Trajectory completed in {elapsed:.3f}s"
                )
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
                return result

            if sys_state == SYS_ERROR:
                self._node.get_logger().error(
                    "STM32 reported error during trajectory"
                )
                result.error_code = (
                    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                )
                goal_handle.abort()
                return result

            if elapsed > timeout:
                self._node.get_logger().error(
                    f"Trajectory timed out after {elapsed:.1f}s "
                    f"(planned {planned_duration:.1f}s)"
                )
                result.error_code = (
                    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                )
                goal_handle.abort()
                return result

            if now - last_keepalive >= 0.2:
                try:
                    self._cmd_client.ping()
                except Exception:
                    pass
                last_keepalive = now

            feedback.desired.time_from_start.sec = int(elapsed)
            feedback.desired.time_from_start.nanosec = int(
                (elapsed - int(elapsed)) * 1e9
            )
            goal_handle.publish_feedback(feedback)

            time.sleep(0.01)

        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        goal_handle.abort()
        return result
