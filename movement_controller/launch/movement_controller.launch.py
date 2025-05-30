from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_file = DeclareLaunchArgument(
        'config_filepath',
        default_value='',
        description='path to config file in bringup'
    )

    return LaunchDescription([
        Node(
            package='movement_controller',
            executable='movement_controller_node',
            name='movement_controller',
            parameters=[LaunchConfiguration('config_filepath')],
            output='screen',
        ),
        config_file
    ])
