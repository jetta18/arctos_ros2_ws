from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package paths
    arctos_description_dir = get_package_share_directory('arctos_description')
    arctos_moveit_dir = get_package_share_directory('arctos_moveit_config')

    # Declare Launch Arguments
    declare_rviz_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=PathJoinSubstitution([arctos_description_dir, "config", "moveit.rviz"]),
        description="Path to RViz configuration file"
    )

    # RViz Configuration
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([arctos_moveit_dir, "config", "arctos.urdf.xacro"]),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Parameters
    ros2_controllers_params = os.path.join(
        arctos_description_dir, 'config', 'ros2_controllers.yaml'
    )

    # NOTE: No separate CAN bridge nodes needed!
    # Our MKS Motor Driver uses direct SocketCAN access via integrated SocketCANBridge

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ROS2 Control Node (Hardware Interface + Controller Manager)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_params],
        output={'stdout': 'screen', 'stderr': 'screen'},
        arguments=[
            '--ros-args',
            '--log-level', 'info',
            # Uncomment for debugging:
            # '--log-level', 'arctos_hardware_interface:=debug',
            # '--log-level', 'controller_manager:=debug'
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_controller", "--controller-manager", "/controller_manager"],
    )

    # Include MoveIt Launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arctos_moveit_dir, "launch", "move_group.launch.py"])
        )
    )

    moveit_config = MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config").to_moveit_configs()

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Delayed controller spawning to ensure proper startup sequence
    # 1. Start control node (hardware interface with direct CAN access)
    # 2. Start joint state broadcaster
    # 3. Start robot controller
    # 4. Start MoveIt and RViz
    
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,  # Wait 3 seconds for hardware interface to initialize
        actions=[joint_state_broadcaster_spawner]
    )
    
    # Ensure robot controller starts after joint state broadcaster
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay rviz and moveit launch until controllers are ready
    delay_rviz_and_moveit_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node, move_group_launch]
        ))
    
    return LaunchDescription([
        declare_rviz_arg,
        LogInfo(msg=["Launching Arctos Robot System..."]),
        
        # Robot description
        robot_state_pub_node,
        
        # Hardware interface and controller manager (direct CAN access)
        control_node,
        
        # Controllers (delayed and sequenced)
        delayed_joint_state_broadcaster,
        delay_robot_controller_spawner,
        
        # MoveIt and RViz (after controllers are ready)
        delay_rviz_and_moveit_launch,
    ])
