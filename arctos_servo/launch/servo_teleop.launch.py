"""Launch MoveIt Servo for the Arctos robot.

Prerequisites (must already be running):
    ros2 launch arctos_bringup arctos_bringup.launch.py

This launch file automatically:
    1. Loads arctos_servo_controller (inactive)
    2. Switches from arctos_controller to arctos_servo_controller
    3. Starts the MoveIt Servo node

After this launch is running, open a second terminal and run:
    ros2 run arctos_servo keyboard_teleop

To switch back to trajectory mode after stopping this launch:
    ros2 control switch_controllers \\
        --deactivate arctos_servo_controller \\
        --activate arctos_controller
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name: str, file_path: str):
    """Load a YAML file from a package share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    with open(absolute_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('arctos', package_name='arctos_moveit_config')
        .to_moveit_configs()
    )

    servo_yaml = load_yaml('arctos_servo', 'config/arctos_servo_config.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    load_servo_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'arctos_servo_controller', '--inactive',
            '-c', '/controller_manager',
        ],
        output='screen',
    )

    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'arctos_controller',
            '--activate', 'arctos_servo_controller',
        ],
        output='screen',
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output='screen',
    )

    switch_after_load = RegisterEventHandler(
        OnProcessExit(
            target_action=load_servo_controller,
            on_exit=[switch_controllers],
        )
    )

    start_servo_after_switch = RegisterEventHandler(
        OnProcessExit(
            target_action=switch_controllers,
            on_exit=[servo_node],
        )
    )

    return LaunchDescription([
        load_servo_controller,
        switch_after_load,
        start_servo_after_switch,
    ])
