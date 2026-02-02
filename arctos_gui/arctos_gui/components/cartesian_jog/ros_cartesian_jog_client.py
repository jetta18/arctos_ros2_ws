"""ROS 2 cartesian jog client using MoveIt MoveGroup action."""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from .cartesian_jog_client_protocol import CartesianJogClient


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple:
    """Convert Euler angles to quaternion (roll, pitch, yaw in radians)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple:
    """Convert quaternion to Euler angles (roll, pitch, yaw in radians)."""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return (roll, pitch, yaw)


class ArctosCartesianJogClient(CartesianJogClient):
    """MoveIt action-based cartesian jog client using MoveGroup action."""

    def __init__(self, node_name: str = "arctos_cartesian_jog_client") -> None:
        self._node_name = node_name
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        
        self._move_group_client: Optional[ActionClient] = None
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        
        self._joint_state_sub = None
        self._current_joint_state: Optional[JointState] = None
        
        self._connected = False
        self._state_lock = threading.Lock()
        
        self._planning_group = "arctos_arm"
        self._ee_link = "Link_6_1"
        self._base_frame = "world"

    def connect(self) -> bool:
        with self._state_lock:
            if self._connected:
                return True

            try:
                if not rclpy.ok():
                    rclpy.init()

                self._node = Node(self._node_name)
                
                self._move_group_client = ActionClient(
                    self._node,
                    MoveGroupAction,
                    "/move_action"
                )
                
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self._node)
                
                self._joint_state_sub = self._node.create_subscription(
                    JointState,
                    "/joint_states",
                    self._joint_state_callback,
                    10
                )
                
                self._executor = MultiThreadedExecutor()
                self._executor.add_node(self._node)
                
                self._spin_thread = threading.Thread(target=self._spin, daemon=True)
                self._spin_thread.start()
                
                self._node.get_logger().info("Waiting for MoveGroup action server...")
                if not self._move_group_client.wait_for_server(timeout_sec=3.0):
                    self._node.get_logger().warn(
                        "MoveGroup action server not available. Make sure MoveIt is running."
                    )
                    self._cleanup()
                    return False
                
                self._node.get_logger().info("Connected to MoveGroup action server")
                self._connected = True
                return True

            except Exception as exc:
                if self._node:
                    self._node.get_logger().error(f"Failed to connect: {exc}")
                self._cleanup()
                return False

    def disconnect(self) -> None:
        with self._state_lock:
            if not self._connected:
                return
            self._connected = False

        self._cleanup()

    def is_connected(self) -> bool:
        with self._state_lock:
            return self._connected

    def get_current_pose(self) -> dict:
        if not self._connected or self._tf_buffer is None:
            return {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            pos = transform.transform.translation
            rot = transform.transform.rotation
            
            rpy = euler_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            
            return {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
                "rx": float(rpy[0]),
                "ry": float(rpy[1]),
                "rz": float(rpy[2]),
            }
        except Exception as exc:
            if self._node:
                self._node.get_logger().debug(f"Failed to get current pose: {exc}")
            return {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}

    def send_cartesian_step(self, axis: str, step_m: float, speed_scale: float) -> bool:
        if not self._connected or self._move_group_client is None or self._node is None:
            return False

        try:
            current_pose_dict = self.get_current_pose()
            
            target_pose = PoseStamped()
            target_pose.header.frame_id = self._base_frame
            target_pose.header.stamp = self._node.get_clock().now().to_msg()
            
            target_pose.pose.position.x = current_pose_dict["x"]
            target_pose.pose.position.y = current_pose_dict["y"]
            target_pose.pose.position.z = current_pose_dict["z"]
            
            # Keep current orientation for all movements
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 1.0
            
            # Get current orientation from TF
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._ee_link,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                target_pose.pose.orientation = transform.transform.rotation
            except Exception:
                # Fallback to identity quaternion
                pass
            
            # Only modify position for translation axes
            if axis == "x":
                target_pose.pose.position.x += step_m
            elif axis == "y":
                target_pose.pose.position.y += step_m
            elif axis == "z":
                target_pose.pose.position.z += step_m
            elif axis == "rx":
                # For rotation, we need to modify orientation
                rpy = [current_pose_dict["rx"], current_pose_dict["ry"], current_pose_dict["rz"]]
                rpy[0] += step_m
                quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
                target_pose.pose.orientation.x = quat[0]
                target_pose.pose.orientation.y = quat[1]
                target_pose.pose.orientation.z = quat[2]
                target_pose.pose.orientation.w = quat[3]
            elif axis == "ry":
                rpy = [current_pose_dict["rx"], current_pose_dict["ry"], current_pose_dict["rz"]]
                rpy[1] += step_m
                quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
                target_pose.pose.orientation.x = quat[0]
                target_pose.pose.orientation.y = quat[1]
                target_pose.pose.orientation.z = quat[2]
                target_pose.pose.orientation.w = quat[3]
            elif axis == "rz":
                rpy = [current_pose_dict["rx"], current_pose_dict["ry"], current_pose_dict["rz"]]
                rpy[2] += step_m
                quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
                target_pose.pose.orientation.x = quat[0]
                target_pose.pose.orientation.y = quat[1]
                target_pose.pose.orientation.z = quat[2]
                target_pose.pose.orientation.w = quat[3]
            else:
                return False
            
            goal = MoveGroupAction.Goal()
            goal.request = MotionPlanRequest()
            goal.request.group_name = self._planning_group
            goal.request.num_planning_attempts = 3
            goal.request.allowed_planning_time = 2.0
            goal.request.max_velocity_scaling_factor = speed_scale
            goal.request.max_acceleration_scaling_factor = speed_scale
            goal.request.planner_id = "LIN"
            
            goal.request.workspace_parameters.header.frame_id = self._base_frame
            goal.request.workspace_parameters.min_corner.x = -1.0
            goal.request.workspace_parameters.min_corner.y = -1.0
            goal.request.workspace_parameters.min_corner.z = -1.0
            goal.request.workspace_parameters.max_corner.x = 1.0
            goal.request.workspace_parameters.max_corner.y = 1.0
            goal.request.workspace_parameters.max_corner.z = 1.0
            
            if self._current_joint_state:
                goal.request.start_state.joint_state = self._current_joint_state
            
            # Position constraint: move to target position
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = self._base_frame
            pos_constraint.link_name = self._ee_link
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            bounding_volume = BoundingVolume()
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.001]
            bounding_volume.primitives.append(sphere)
            bounding_volume.primitive_poses.append(target_pose.pose)
            pos_constraint.constraint_region = bounding_volume
            pos_constraint.weight = 1.0
            
            # Orientation constraint: keep current orientation exactly
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = self._base_frame
            orient_constraint.link_name = self._ee_link
            orient_constraint.orientation = target_pose.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.001  # Very tight tolerance
            orient_constraint.absolute_y_axis_tolerance = 0.001  # Very tight tolerance
            orient_constraint.absolute_z_axis_tolerance = 0.001  # Very tight tolerance
            orient_constraint.weight = 1.0
            
            constraints = Constraints()
            constraints.position_constraints.append(pos_constraint)
            constraints.orientation_constraints.append(orient_constraint)
            goal.request.goal_constraints.append(constraints)
            
            goal.planning_options = PlanningOptions()
            goal.planning_options.plan_only = False
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            
            self._node.get_logger().info(f"Sending cartesian step: {axis} {step_m:.3f}m")
            
            future = self._move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            
            if not future.done():
                self._node.get_logger().error("Goal send timeout")
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._node.get_logger().error("Goal rejected by action server")
                return False
            
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=10.0)
            
            if not result_future.done():
                self._node.get_logger().error("Execution timeout")
                return False
            
            result = result_future.result()
            if result.result.error_code.val == 1:
                self._node.get_logger().info("Cartesian step executed successfully")
                return True
            else:
                self._node.get_logger().error(f"Execution failed with error code: {result.result.error_code.val}")
                return False

        except Exception as exc:
            if self._node:
                self._node.get_logger().error(f"Cartesian step failed: {exc}")
            return False

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._state_lock:
            self._current_joint_state = msg

    def _spin(self) -> None:
        if self._executor is None:
            return

        try:
            self._executor.spin()
        except Exception as exc:
            if self._node:
                self._node.get_logger().error(f"ROS executor spin failed: {exc}")

    def _cleanup(self) -> None:
        if self._executor:
            self._executor.shutdown()
            self._executor = None

        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None

        if self._joint_state_sub:
            try:
                self._node.destroy_subscription(self._joint_state_sub)
            except Exception:
                pass
            self._joint_state_sub = None

        if self._tf_listener:
            self._tf_listener = None

        if self._tf_buffer:
            self._tf_buffer = None

        if self._move_group_client:
            self._move_group_client = None

        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
