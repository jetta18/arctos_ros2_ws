from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Arctos GUI Node (MKS config now uses direct CAN communication)
    arctos_gui_node = Node(
        package='arctos_gui',
        executable='arctos_gui',
        name='arctos_gui',
        output='screen'
    )
    
    return LaunchDescription([
        arctos_gui_node
    ])
