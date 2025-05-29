from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    from ament_index_python.packages import get_package_share_directory

    camera_pkg = get_package_share_directory('megaProsjektCameraPkg')
    robot_pkg = get_package_share_directory('movement_controller')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_pkg, 'launch', 'camera.launch.py')
        )
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, 'launch', 'robot.launch.py')
        )
    )

    return LaunchDescription([
        camera_launch,
        robot_launch
    ])