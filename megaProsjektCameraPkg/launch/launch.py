from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare the video device argument with default value
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video4',
            description='Path to the video device (e.g., /dev/video0)'
        ),

        DeclareLaunchArgument(
            'camera_height',
            default_value='30.0',  # Default value as string
            description='Height of the camera in cm'
        ),

        DeclareLaunchArgument(
            'space_angle',
            default_value='0.0',  # Default value as string
            description='rotation of the entire coordinate system around the cameras normal axis'
        ),
        
        # USB Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
            }],
            output='screen'
        ),
        
        # Gaussian Blur Node
        Node(
            package='megaProsjektCameraPkg',  # Replace with your actual package name
            executable='visionNode',
            name='vision_node',
            parameters=[{
                'camera_height': LaunchConfiguration('camera_height'),
                'space_angle': LaunchConfiguration('space_angle'),
            }],
            output='screen'
        )
    ])