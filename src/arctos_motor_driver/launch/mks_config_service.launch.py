from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name (e.g., can0, can1)'
    )
    
    mks_config_service_node = Node(
        package='arctos_motor_driver',
        executable='mks_config_service',
        name='mks_config_service',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface')
        }]
    )
    
    return LaunchDescription([
        can_interface_arg,
        mks_config_service_node
    ])
