import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


def rotate_point(x, y, angle_degrees):
    """Rotate a 2D point (x, y) counter-clockwise around the origin by a given angle (in degrees)."""
    theta = math.radians(angle_degrees)  # Convert angle to radians
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    
    # Apply rotation matrix
    x_rotated = x * cos_theta - y * sin_theta
    y_rotated = x * sin_theta + y * cos_theta
    
    return (x_rotated, y_rotated)

class ColorDetectionNode(Node):
    """
    A node for detecting red and blue objects in an image
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the color detection node.
        """
        super().__init__('color_detection')

        self.declare_parameter('camera_height', 30.0)  # Default value is 30.0

        self.declare_parameter('space_angle', 0.0)  # Default value is 0.0

        self.declare_parameter('home_pos_x', 0.2)
        self.declare_parameter('home_pos_z', 0.2)
        
        # Get the parameter value
        self.cameraHeight = self.get_parameter('camera_height').get_parameter_value().double_value

        self.angle = self.get_parameter('space_angle').get_parameter_value().double_value

        self.home_x = self.get_parameter('home_pos_x').get_parameter_value().double_value
        self.home_z = self.get_parameter('home_pos_z').get_parameter_value().double_value

        self.get_logger().info(f"Camera height set to: {self.cameraHeight}")


        self.resolutionX = 640
        self.resolutionY = 480
        self.aspectRatio = self.resolutionY/self.resolutionX
        self.width = 93.5*0.5765*(self.cameraHeight/30)
        self.height = self.width*self.aspectRatio

        # Subscribe to an image topic
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the processed image
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
        
        self.position_pub = self.create_publisher(
            Float32MultiArray, 
            'detected_positions', 
            10)
  
        # Initialize CVBridge
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        """
        Callback function for input image topic.
        Detects red and blue objects and draws circles around them.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        
        # Convert to HSV color space for better color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for red and blue
        # Red color range (note: red wraps around 0 in HSV)
        lower_red1 = np.array([0, 110, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 110, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Blue color range
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([130, 255, 255])

        # Blue color range
        lower_green = np.array([30, 25, 25])
        upper_green = np.array([80, 255, 255])
        
        # Create masks for red and blue
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        detected_positions = []

        def extendPos(x,y):
            x_pos = x/self.resolutionX
            x_pos -= 0.5
            x_pos *= self.width
            y_pos = y/self.resolutionY
            y_pos -= 0.5
            y_pos *= self.height
            x_pos, y_pos = rotate_point(x_pos,y_pos,self.angle)
            x_pos = self.home_x+x_pos/100.0
            y_pos = self.home_z-y_pos/100.0
            detected_positions.extend([x_pos, y_pos])
        
        # Find contours for red objects
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_red:
            area = cv2.contourArea(cnt)
            if area > 500:  # Filter small objects
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(cv_image, center, radius, (0, 0, 255), 2)  # Red circle
                if(len(detected_positions) == 0):
                    extendPos(x,y)
                    
        
        # Find contours for blue objects
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_blue:
            area = cv2.contourArea(cnt)
            if area > 500:  # Filter small objects
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(cv_image, center, radius, (255, 0, 0), 2)  # Blue circle
                if(len(detected_positions) == 2):
                    extendPos(x,y)

        # Find contours for blue objects
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_green:
            area = cv2.contourArea(cnt)
            if area > 500:  # Filter small objects
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(cv_image, center, radius, (0, 255, 0), 2)  # Blue circle
                if(len(detected_positions) == 4):
                    extendPos(x,y)

        # Publish positions (add this)
        if len(detected_positions) == 6:
            self.positions = detected_positions
            position_msg = Float32MultiArray()
            position_msg.data = detected_positions
            self.position_pub.publish(position_msg)
        
        # Convert back to ROS Image message
        try:
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
  
        # Publish the processed image
        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()