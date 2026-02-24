from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, TimerAction, EmitEvent, LogInfo,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import os


def generate_launch_description():
    arctos_description_dir = get_package_share_directory('arctos_description')
    arctos_moveit_dir = get_package_share_directory('arctos_moveit_config')
    arctos_bridge_dir = get_package_share_directory('arctos_bridge')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([arctos_description_dir, "urdf", "arctos.urdf.xacro"]),
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ros2_controllers_params = os.path.join(
        arctos_description_dir, 'config', 'ros2_controllers.yaml'
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    arctos_bridge_config = os.path.join(arctos_bridge_dir, 'config', 'arctos_bridge.yaml')

    arctos_bridge_node = Node(
        package="arctos_bridge",
        executable="arctos_bridge_node",
        name="arctos_bridge_node",
        parameters=[arctos_bridge_config],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_params],
        output={'stdout': 'screen', 'stderr': 'screen'},
        arguments=[
            '--ros-args',
            '--log-level', 'info',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    use_gripper_arg = DeclareLaunchArgument(
        'use_gripper', default_value='true',
        description='Spawn the gripper controller (set false to disable)'
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_controller", "--controller-manager", "/controller_manager"],
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_servo_controller", "--inactive", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration('use_gripper')),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arctos_moveit_dir, "launch", "move_group.launch.py"])
        )
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arctos_moveit_dir, "launch", "moveit_rviz.launch.py"])
        )
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_servo_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[servo_controller_spawner],
        )
    )

    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=servo_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    delay_rviz_and_moveit_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[moveit_rviz_launch, move_group_launch]
        )
    )

    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[
                LogInfo(msg=["ros2_control_node exited. Shutting down launch..."]),
                EmitEvent(event=Shutdown(reason="ros2_control_node exited")),
            ],
        )
    )

    delayed_control_node = TimerAction(
        period=2.0,
        actions=[control_node],
    )

    return LaunchDescription([
        use_gripper_arg,
        LogInfo(msg=["Launching Arctos Robot System..."]),

        arctos_bridge_node,
        robot_state_pub_node,
        delayed_control_node,
        shutdown_on_control_exit,

        delayed_joint_state_broadcaster,
        delay_robot_controller_spawner,
        delay_servo_controller_spawner,
        delay_gripper_controller_spawner,

        delay_rviz_and_moveit_launch,
    ])
