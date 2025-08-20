from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arctos_step_bridge',
            executable='step_bridge_node',
            name='step_bridge',
            output='screen',
            parameters=[{
                # Input topic from topic_based_ros2_control
                'input_topic': '/joint_commands_radian',
                # Expected joint order
                'joint_names': ['X_joint','Y_joint','Z_joint','A_joint','B_joint','C_joint'],
                # Mechanics (defaults reflect ESP32 firmware)
                'steps_per_rev': [200.0, 200.0, 200.0, 200.0, 200.0, 200.0],
                'microstepping': [16.0, 16.0, 16.0, 16.0, 16.0, 16.0],
                'gear_ratio': [13.5, 150.0, 150.0, 48.0, 27.3375, 10.0],
            }]
        )
    ])
