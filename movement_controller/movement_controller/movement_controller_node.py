import rclpy
from rclpy.node import Node
# from rclpy.action import ActionClient
# from moveit_msgs.action import MoveGroup
# from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
# from moveit_msgs.srv import GetPositionFK
# from geometry_msgs.msg import PoseStamped
# from shape_msgs.msg import SolidPrimitive
# from geometry_msgs.msg import _pose_with_covariance_stamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import SetParametersResult

import time
import threading


class Movement_controller(Node):
    def __init__(self):
        super().__init__('movement_controller_node')
        self.get_logger().info("Started movement controller node")

        self.detected_pos_sub = self.create_subscription(
            Float32MultiArray,
            '/detected_positions',
            self.callback_receive_detected_positions,
            10
        )

        self.target_pos_pub = self.create_publisher(
            Point,
            '/target_position', 
            10
        )

        self.declare_parameter('search_size', 0.2)
        self.declare_parameter('move_time', 4.0)
        self.declare_parameter('target_height', 0.09)

        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Get initial values
        self.search_size = self.get_parameter('search_size').value
        self.move_time = self.get_parameter('move_time').value
        self.target_height = self.get_parameter('target_height').value
        
        self.build_search_pattern() 
        self.receive_detected_positions = False # Bool for accepting new cube positions from vision node (between movements)
        self.detected_positions = None
        self.searching_for_cubes = True
        self.current_search_index = 0
        self.message_received_event = threading.Event()
        self.run_controller()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'move_time':
                self.move_time = param.value
                self.get_logger().info(f'Move time: {self.move_time}')
            elif param.name == 'search_size':
                self.search_size = param.value
                self.build_search_pattern()
                self.get_logger().info(f'Search size: {self.search_size}')
            elif param.name == 'target_height':
                self.target_height = param.value
                self.get_logger().info(f'Target height (m): {self.target_height}')
        
        return SetParametersResult(successful=True)
    
    def build_search_pattern(self):
        """Create search positions"""
        center_x, center_y = -0.35, 0.45
        self.search_positions = [
            Point(x=center_x - self.search_size, y=center_y - self.search_size, z=0.5),
            Point(x=center_x + self.search_size, y=center_y - self.search_size, z=0.5),
            Point(x=center_x + self.search_size, y=center_y + self.search_size, z=0.5),
            Point(x=center_x - self.search_size, y=center_y + self.search_size, z=0.5),
        ]
        
    def callback_receive_detected_positions(self, msg):
        if self.receive_detected_positions:
            self.detected_positions = msg
            self.message_received_event.set()  # Signal that message arrived

    def search_loop(self):
        """Moves in a square pattern to search for cubes"""
        while self.searching_for_cubes:
            # Move to next search position
            current_pos = self.search_positions[self.current_search_index]
            self.get_logger().info(f"Moving to search position {self.current_search_index}")
            self.target_pos_pub.publish(current_pos)
            time.sleep(self.move_time)  # Wait for movement
            # Update current position
            current_search_pos = self.search_positions[self.current_search_index]
            
            # Pause and listen for detections
            self.get_logger().info("Pausing to search for cubes...")
            self.receive_detected_positions = True
            self.message_received_event.clear()
            
            if self.message_received_event.wait(timeout=1.0):
                # Cubes found! Use correct reference position
                self.get_logger().info("Cubes found")
                self.receive_detected_positions = False
                self.send_target_movements(self.detected_positions, current_search_pos)
            else:
                # No cubes detected, continue to next position
                self.get_logger().info("No cubes detected, moving to next position...")
                self.receive_detected_positions = False
                
            # Move to next search position
            self.current_search_index = (self.current_search_index + 1) % len(self.search_positions)

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
                cube_pos.z = self.target_height
                target_positions.append(cube_pos)

        # Go back to the position the robot was in when it saw the cubes
        target_positions.append(reference_position)   

        for pos in target_positions:
            self.target_pos_pub.publish(pos)
            self.get_logger().info(f"Published target_position: x={pos.x}, y={pos.y}, z={pos.z}")
            time.sleep(self.move_time)   #################################### Lage parameter som kan endres. Prøve om det funker uten sleep i det hele tatt 

    def run_controller(self):       
        self.target_pos_pub.publish(self.search_positions[0])
        self.current_search_index += 1
        # Start search 
        threading.Timer(5.0, lambda: (
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