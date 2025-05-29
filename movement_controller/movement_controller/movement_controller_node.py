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
import threading

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

class Movement_controller(Node):
    def __init__(self):
        super().__init__('movement_controller_node')
        self.get_logger().info("Started movement controller node")

        ########################### Lage subscription til goal pose
        self.detected_pos_sub = self.create_subscription(
            Float32MultiArray,             # Message type: geometry_msgs/msg/Point
            '/detected_positions',         # Same topic name you used
            self.callback_receive_detected_positions,        # Callback function
            10                          # Queue size
        )

        self.target_pos_pub = self.create_publisher(
            Point,                      # Message type
            '/target_position',         # Topic name
            10                          # Queue size
        )

        # Add home pos to go back between each cube
        self.home_pos = Point()
        self.home_pos.x = -0.2
        self.home_pos.y = 0.5
        self.home_pos.z = 0.5

        # Bool for accepting new cube positions from vision node
        self.receive_detected_positions = False
        self.detected_positions = None
        self.searching_for_cubes = True

        self.message_received_event = threading.Event()

        # Square search pattern
        # self.search_positions = [
        #     Point(x=-0.15, y=0.3, z=0.5),
        #     Point(x=-0.4, y=0.3, z=0.5),
        #     Point(x=-0.4, y=0.6, z=0.5),
        #     Point(x=-0.15, y=0.6, z=0.5),
        # ]
        self.search_positions = [
            Point(x=-0.2, y=0.5, z=0.5),
            Point(x=-0.2, y=0.5, z=0.5),
            Point(x=-0.2, y=0.5, z=0.5),
            Point(x=-0.2, y=0.5, z=0.5),
        ]
        self.current_search_index = 0

        self.run_controller()
        
    def callback_receive_detected_positions(self, msg):
        if self.receive_detected_positions:
            self.detected_positions = msg
            self.message_received_event.set()  # Signal that message arrived

    def search_loop(self):
        #current_search_pos = self.home_pos
        while self.searching_for_cubes:
            current_search_pos = self.search_positions[self.current_search_index]

            # Now pause and listen for detections
            self.get_logger().info("Pausing to search for cubes...")
            self.receive_detected_positions = True
            self.message_received_event.clear()
            
            # Wait 1 second for detection message
            if self.message_received_event.wait(timeout=0.8):
                # Message received!
                self.get_logger().info("Cubes found! Stopping search.")
                #self.searching_for_cubes = False
                self.receive_detected_positions = False
                self.send_target_movements(self.detected_positions, current_search_pos)
            else:
                # Timeout - no cubes detected, continue search
                self.get_logger().info("No cubes detected, moving to next position...")
                self.receive_detected_positions = False
                # Move to next search position
                current_pos = self.search_positions[self.current_search_index]
                self.get_logger().info(f"Moving to search position {self.current_search_index}")
                self.target_pos_pub.publish(current_pos)
                # Wait for movement to complete 
                time.sleep(8)  # Movement time
                self.current_search_index = (self.current_search_index + 1) % len(self.search_positions)
            
            if not self.searching_for_cubes:  # Check if we should stop
                break
            
        
        # If we've searched all positions, restart or stop
        if self.searching_for_cubes:    
            self.get_logger().info("Completed search pattern, restarting...")
            self.current_search_index = 0

    def send_target_movements(self, msg, reference_position):
        """Callback that receives detected_position and publishes to target_position"""
        self.get_logger().info(f"Received detected_positions: {msg.data}")

        # Add a tuple with x and y for each 
        target_positions = []
        for i in range(0, len(msg.data), 2):  
            if i + 1 < len(msg.data):
                # Target cube position
                cube_pos = Point()  # Create new Point object
                cube_pos.x = reference_position.x + float(msg.data[i])
                cube_pos.y = reference_position.y + float(msg.data[i + 1])
                cube_pos.z = 0.09
                target_positions.append(cube_pos)

        # Go back to the position the robot was in when it saw the cubes
        target_positions.append(reference_position)   

                # Move back to home pos between pointing movements
                ###########target_positions.append(self.home_pos)

        for pos in target_positions: 
            self.target_pos_pub.publish(pos)
            self.get_logger().info(f"Published target_position: x={pos.x}, y={pos.y}, z={pos.z}")
            time.sleep(5)   #################################### Lage parameter som kan endres. Prøve om det funker uten sleep i det hele tatt 

    def run_controller(self):       
        self.target_pos_pub.publish(self.search_positions[0])
        self.current_search_index += 1
        # Start search 
        threading.Timer(8.0, lambda: (
            self.get_logger().info("Starting cube search pattern..."),
            threading.Thread(target=self.search_loop, daemon=True).start()
        )).start()

def main():
    rclpy.init()
    movement_controller = Movement_controller()
    rclpy.spin(movement_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()