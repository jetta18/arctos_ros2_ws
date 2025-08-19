from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_topic_based = LaunchConfiguration('use_topic_based')
    joint_commands_topic = LaunchConfiguration('joint_commands_topic')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    controllers_yaml = LaunchConfiguration('controllers_yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_topic_based', default_value='true'),
        DeclareLaunchArgument('joint_commands_topic', default_value='/topic_based_joint_commands'),
        DeclareLaunchArgument('joint_states_topic', default_value='/topic_based_joint_states'),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('arctos_description'), 'config', 'ros2_controllers.yaml'
            ]),
        ),

        # Robot description with ros2_control tag; pass xacro args to enable topic-based plugin
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([FindPackageShare('arctos_description'), 'urdf', 'arctos.urdf.xacro']),
                    ' use_topic_based:=', use_topic_based,
                    ' joint_commands_topic:=', joint_commands_topic,
                    ' joint_states_topic:=', joint_states_topic,
                ])
            }],
        ),

        # Controller manager with controllers config
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([FindPackageShare('arctos_description'), 'urdf', 'arctos.urdf.xacro']),
                    ' use_topic_based:=', use_topic_based,
                    ' joint_commands_topic:=', joint_commands_topic,
                    ' joint_states_topic:=', joint_states_topic,
                ])},
                controllers_yaml,
            ],
            output='screen',
        ),

        # Spawners for controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arctos_controller'],
            output='screen',
        ),

        # Optional: start simulator node to emulate ESP32 behavior
        Node(
            package='arctos_control',
            executable='sim_esp32',
            name='sim_esp32',
            parameters=[PathJoinSubstitution([FindPackageShare('arctos_control'), 'config', 'topic_based.yaml'])],
            output='screen',
        ),

        # Optional watchdog
        Node(
            package='arctos_control',
            executable='watchdog',
            name='command_watchdog',
            parameters=[{'timeout_sec': 1.0}],
            output='screen',
        ),
    ])
