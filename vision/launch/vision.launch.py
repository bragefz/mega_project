from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    config_file = DeclareLaunchArgument(
        'config_filepath',
        default_value = '',
        description = 'path to config file in bringup'
    )

    usb_cam_node = Node(
        package = 'usb_cam',
        executable = 'usb_cam_node_exe',
        name = 'usb_cam_node',
        parameters=[LaunchConfiguration('config_filepath')],
        output='screen'
    )

    vision_node = Node(
        package='vision',
        executable='vision_node',
        name='vision_node',
        parameters=[LaunchConfiguration('config_filepath')],
        output='screen'
    )

    return LaunchDescription([
        config_file,
        usb_cam_node,
        vision_node
    ])