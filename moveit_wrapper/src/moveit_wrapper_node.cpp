#include <memory>  
#include <rclcpp/rclcpp.hpp>  
#include <geometry_msgs/msg/point.hpp>  
#include <tf2/LinearMath/Quaternion.h> // Quaternion operations for orientation representation
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // Conversion between TF2 and ROS geometry messages
#include <moveit/move_group_interface/move_group_interface.hpp>  // MoveIt interface for robot control



// Create a logger instance for this node to output messages
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_wrapper_node");

// Main node class for UR5 robot control
class UR5SimpleController : public rclcpp::Node {
public:
    // Constructor with node configuration options
    UR5SimpleController(const rclcpp::NodeOptions& options)
        : Node("moveit_wrapper_node", options),  // Initialize base Node class with name
          // Initialize MoveIt interface for the UR5 manipulator arm (move group)
          move_group(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator")
    {
        // Configure MoveIt planning parameters:
        move_group.setPlanningTime(10.0);  // Maximum time allowed for planning (seconds)
        move_group.setNumPlanningAttempts(50);  // Number of planning attempts before failing
        move_group.setGoalPositionTolerance(0.0005);  // Acceptable position error (meters)
        move_group.setGoalOrientationTolerance(0.0005);  // Acceptable orientation error (radians)

        // Create a subscription to receive target positions:
        pos_sub = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_position",  // Topic name to subscribe to
            10,  // Message queue size
            // Callback function when new message arrives
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                position_goal_callback(msg);  // Forward to callback method
            });

        // Log initialization message
        RCLCPP_INFO(LOGGER, "UR5 Simple Controller initialized");
    }

private:
    // MoveIt interface instance for controlling the robot arm
    moveit::planning_interface::MoveGroupInterface move_group;
    
    // ROS subscription for receiving target positions
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pos_sub;

    // Callback function that handles incoming position commands
    void position_goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // Log the received position
        RCLCPP_INFO(LOGGER, "Received new position goal: x=%.2f, y=%.2f, z=%.2f", 
                   msg->x, msg->y, msg->z);

        try {
            // Create a target pose message for MoveIt
            geometry_msgs::msg::Pose target_pose;
            
            // Set the position components from the incoming message
            target_pose.position.x = msg->x;
            target_pose.position.y = msg->y;
            target_pose.position.z = msg->z;

            // Set a fixed orientation (currently 180° around Z-axis)
            // Note: This creates an invalid quaternion (length = 0)
            target_pose.orientation.x = 0.0;
            target_pose.orientation.y = 1.0;
            target_pose.orientation.z = 0.0;  // (0, 0, 1, 0) would be 180° Z-axis
            target_pose.orientation.w = 0.0;

            // Set the target pose for the robot arm
            move_group.setPoseTarget(target_pose);            
            
            // Create a plan object to store the motion plan
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            
            // Attempt to generate a motion plan
            if (move_group.plan(plan)) {
                // If planning succeeds, execute the motion
                auto result = move_group.execute(plan);
                
                // Check execution result
                if(result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(LOGGER, "Movement successful: ");
                } else {
                    RCLCPP_ERROR(LOGGER, "Execution failed: %d", result.val);
                }
            } else {
                // Planning failed
                RCLCPP_ERROR(LOGGER, "Failed planning");
            }
        } catch (const std::exception& e) {
            // Handle any exceptions during planning or execution
            RCLCPP_ERROR(LOGGER, "Movement error: %s", e.what());
        }
    }
};

// Main function
int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create node configuration options
    rclcpp::NodeOptions node_options;
    // Enable automatic parameter declaration from command line overrides
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // Create and run the node instance
    auto node = std::make_shared<UR5SimpleController>(node_options);
    // Keep the node running and processing callbacks
    rclcpp::spin(node);
    
    // Clean shutdown when done
    rclcpp::shutdown();
    return 0;
}