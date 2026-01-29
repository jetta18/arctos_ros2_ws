from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paketpfade
    arctos_description_dir = get_package_share_directory('arctos_description')

    # URDF via xacro generieren
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

    # ros2_control-Konfiguration
    ros2_controllers_params = os.path.join(
        arctos_description_dir, 'config', 'ros2_controllers.yaml'
    )

    # 1) Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 2) ros2_control_node (Hardware-Interface + Controller Manager)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_params],
        output={'stdout': 'screen', 'stderr': 'screen'},
        arguments=[
            '--ros-args',
            '--log-level', 'info',
            # z.B. zum Debuggen:
            # '--log-level', 'STM32StepperInterface:=debug',
            # '--log-level', 'controller_manager:=debug',
        ],
    )

    # 3) joint_state_broadcaster spawnen
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 4) Joint Trajectory Controller (arctos_controller) spawnen
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 5) Deine Arctos GUI Node
    arctos_gui_node = Node(
        package='arctos_gui',
        executable='arctos_gui',
        name='arctos_gui',
        output='screen',
    )

    # Zeitliche Staffelung:
    # - erst control_node starten
    # - nach 3 s joint_state_broadcaster
    # - nach 5 s arctos_controller
    # - nach 7 s GUI

    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_trajectory_controller = TimerAction(
        period=5.0,
        actions=[trajectory_controller_spawner],
    )

    delayed_gui = TimerAction(
        period=7.0,
        actions=[arctos_gui_node],
    )

    return LaunchDescription([
        LogInfo(msg=["Launching Arctos GUI with own ros2_control_node..."]),

        # Robot Description + ros2_control_node
        robot_state_pub_node,
        control_node,

        # Controller-Spawner (zeitlich verzögert)
        delayed_joint_state_broadcaster,
        delayed_trajectory_controller,

        # GUI (nachdem Controller aktiv sein sollten)
        delayed_gui,
    ])
