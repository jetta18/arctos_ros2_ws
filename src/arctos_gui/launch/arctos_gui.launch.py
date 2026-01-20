from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name for MKS motor driver (e.g., can0, can1)'
    )
    
    enable_mks_config_arg = DeclareLaunchArgument(
        'enable_mks_config',
        default_value='true',
        description='Enable MKS motor configuration service'
    )
    
    # MKS Config Service Node
    mks_config_service_node = Node(
        package='arctos_motor_driver',
        executable='mks_config_service',
        name='mks_config_service',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface')
        }],
        condition=IfCondition(LaunchConfiguration('enable_mks_config'))
    )
    
    # Arctos GUI Node
    arctos_gui_node = Node(
        package='arctos_gui',
        executable='arctos_gui',
        name='arctos_gui',
        output='screen'
    )
    
    return LaunchDescription([
        can_interface_arg,
        enable_mks_config_arg,
        mks_config_service_node,
        arctos_gui_node
    ])
