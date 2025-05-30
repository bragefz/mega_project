from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ur5_controller_node = Node(
        package = 'moveit_wrapper',
        executable = 'moveit_wrapper_node',
        name = 'ur5_controller_node',
        output = 'screen'
    )

    return LaunchDescription([
        ur5_controller_node
    ])