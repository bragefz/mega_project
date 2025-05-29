from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='movement_controller',
            executable='Movement_manager',
            name='movement_controller',
            output='screen',
        )
    ])
