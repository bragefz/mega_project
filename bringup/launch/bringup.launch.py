from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'params.yaml'
    )

    camera_pkg = get_package_share_directory('vision')
    robot_pkg = get_package_share_directory('movement_controller')
    wrapper_pkg = get_package_share_directory('moveit_wrapper')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_pkg, 'launch', 'vision.launch.py')
        ),
        launch_arguments={
            'config_filepath': config
        }.items()
    )

    wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wrapper_pkg, 'launch', 'wrapper.launch.py')
        )
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, 'launch', 'movement_controller.launch.py')
        ),
        launch_arguments={
            'config_filepath': config
        }.items()
    )

    return LaunchDescription([
        camera_launch,
        wrapper_launch,
        robot_launch,
    ])