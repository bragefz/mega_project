from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ur5_controller_node = Node(
        package = 'ur5_simple_controller',
        executable = 'ur5_simple_controller',
        name = 'ur5_controller_node',
        output = 'screen'
    )

    return LaunchDescription([
        ur5_controller_node
    ])