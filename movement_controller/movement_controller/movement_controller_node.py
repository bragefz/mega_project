import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import _pose_with_covariance_stamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

import time

#from moveit.planning import 

from tf_transformations import quaternion_from_euler
from transforms3d.euler import euler2quat
from scipy.spatial.transform import Rotation as R #Nyere enn euler2quat?

"""Used to change between reference frames, will be useful for switching from camera to EE ref"""
# #from tf2_geometry_msgs import do_transform_point
# import tf2_geometry_msgs
# ppp = tf2_geometry_msgs.p
# import geometry_msgs.msg
# geometry_msgs.msg.p

class Movement_manager(Node):
    def __init__(self):
        super().__init__('movement_manager')
        self.get_logger().info("Started movement manager node")

        ########################### Lage subscription til goal pose
        self.detected_pos_sub = self.create_subscription(
            Float32MultiArray,                      # Message type: geometry_msgs/msg/Point
            '/detected_positions',         # Same topic name you used
            self.target_callback,        # Callback function
            10                          # Queue size
        )

        self.target_pos_pub = self.create_publisher(
            Point,                      # Message type
            '/target_position',         # Topic name
            10                          # Queue size
        )
        
        self.do_once = True


    def target_callback(self, msg):
        """Callback that receives detected_position and publishes to target_position"""
        self.get_logger().info(f"Received detected_position: {msg.data}")

        if self.do_once:
            self.do_once = False

            # Add a tuple with x and y for each 
            target_positions = []
            for i in range(0, len(msg.data), 2):  
                if i + 1 < len(msg.data):
                    # Target cube position
                    cube_pos = Point()  # Create new Point object
                    cube_pos.x = float(msg.data[i])
                    cube_pos.y = float(msg.data[i + 1])
                    cube_pos.z = 0.13  # No need for float() on literals
                    target_positions.append(cube_pos)

                    # Add home pos to go back between each cube
                    home_pos = Point()
                    home_pos.x = -0.2
                    home_pos.y = 0.5
                    home_pos.z = 0.5
                    target_positions.append(home_pos)


            for pos in target_positions:
                self.target_pos_pub.publish(pos)
                self.get_logger().info(f"Published target_position: x={pos.x}, y={pos.y}, z={pos.z}")
                time.sleep(4)   #################################### Lage parameter som kan endres. Prøve om det funker uten sleep i det hele tatt 

def main():
    rclpy.init()
    movement_manager = Movement_manager()
    rclpy.spin(movement_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()