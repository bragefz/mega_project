# mega_project
Four packages to control a UR5e robot with vision for detecting colored cubes. 

The package uses 3 nodes: Vision_node, movement_controller_node, and moveit_wrapper_node. 
The vision_node subscribes to the image_raw topic, and uses OpenCV to detect clusters of red, yellow, and blue, 
to publish them as a Float32MultiArray on the detected_positions topic.
The movement_controller_node reads the detected positions, and publishes new target positions to the target_position topic.
The moveit_wrapper_node is a node to abstract away the UR5e robot interface into a single target position topic.

The package has a single unified bringup launch, which launches all required nodes. 
It has 3 launch parameters: video_device, space_angle, and camera_height.
camera_height is the distance from the table to the camera lens in cm, and is used to transform from pixel space to real space.
space_angle is used to rotate the space around the camera's normal axis
video_device is used to select the usb camera to recieve video from.
