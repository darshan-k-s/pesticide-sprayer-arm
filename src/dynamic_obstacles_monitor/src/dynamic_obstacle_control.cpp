#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

/**
 * @brief Node that subscribes to CollisionObject messages and applies them to the MoveIt Planning Scene.
 * * This node acts as a bridge between an external perception system (like YOLO) 
 * and the MoveIt planning environment. It uses the PlanningSceneInterface, 
 * which internally handles the communication (via services or topics) with the 
 * move_group node to update the world model, ensuring obstacles are considered 
 * during motion planning.
 */
class PlanningSceneBridge : public rclcpp::Node
{
public:
    PlanningSceneBridge() : Node("planning_scene_bridge")
    {
        // 1. Initialize Planning Scene Interface
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt PlanningSceneInterface...");
        // The interface is typically ready shortly after the move_group node starts.
        std::this_thread::sleep_for(1s); 

        // 2. Create a Subscriber
        // The topic name is: 'obsFromImg'
        // The message type is moveit_msgs::msg::CollisionObject, as this message 
        // already contains the ID, geometry, pose, and crucial 'operation' field (ADD/REMOVE).
        obstacle_sub_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
            "obsFromImg", 10, std::bind(&PlanningSceneBridge::obstacle_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to 'obsFromImg' and ready to update the planning scene.");
        RCLCPP_INFO(this->get_logger(), "Operation field keys: 0=ADD, 1=REMOVE");
    }

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr obstacle_sub_;

    /**
     * @brief Callback function executed whenever a new CollisionObject is received.
     * * @param msg The received CollisionObject message from the perception system.
     */
    void obstacle_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
    {
        // The CollisionObject message contains the 'operation' field (uint8) which
        // tells MoveIt whether to ADD or REMOVE the object.
        std::string operation_str;
        
        switch (msg->operation) {
            case moveit_msgs::msg::CollisionObject::ADD: // Value 0
                operation_str = "ADDING";
                break;
            case moveit_msgs::msg::CollisionObject::REMOVE: // Value 1
                operation_str = "REMOVING";
                break;
            default:
                operation_str = "UNKNOWN OPERATION FOR";
        }
        // Just use ADD again to move object

        RCLCPP_INFO(this->get_logger(), 
            "Received object '%s' with operation: %s (%d) in frame '%s'.", 
            msg->id.c_str(), operation_str.c_str(), msg->operation, msg->header.frame_id.c_str());

        // The PlanningSceneInterface::applyCollisionObject is the most versatile 
        std::vector<moveit_msgs::msg::CollisionObject> objects_to_apply;
        objects_to_apply.push_back(*msg);
        
        // This call is synchronous and waits until MoveIt confirms the scene update.
        // For a high-rate perception system, you might consider using 
        // applyCollisionObjects(objects_to_apply) and letting the interface batch 
        // the updates if latency is a concern.
        planning_scene_interface_.applyCollisionObjects(objects_to_apply);

        RCLCPP_DEBUG(this->get_logger(), "Successfully applied scene update for object '%s'.", msg->id.c_str());
    }
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Start the node and spin
    rclcpp::spin(std::make_shared<PlanningSceneBridge>());
    
    // Shutdown when spin returns
    rclcpp::shutdown();
    return 0;
}

/*
How to Use This Node

To test this, you would perform the following steps:

1.  **Build the Node:** (Requires `moveit_ros_planning_interface` dependency in your `package.xml`).
2.  **Run MoveIt:** Launch your robot's MoveIt configuration (which starts the `move_group` node).
3.  **Run the Bridge Node:**
    ```bash
    ros2 run [your_package_name] planning_scene_bridge
    ```
4.  **Simulate YOLO Detection (Publish Obstacle):**
    * **ADD a box:** Use the corrected command below. Notice we are publishing to `/obsFromImg`, not `/collision_object`.
        ```bash
        ros2 topic pub --once /obsFromImg moveit_msgs/msg/CollisionObject '{
          header: {frame_id: "world"}, 
          id: "yolo_box_001", 
          operation: [0], 
          primitives: [{type: 1, dimensions: [0.3, 0.3, 0.3]}], 
          primitive_poses: [{position: {x: 0.5, y: -0.5, z: 0.15}, orientation: {w: 1.0}}]
        }'
        ```
    * **REMOVE the box:**
        ```bash
        ros2 topic pub --once /obsFromImg moveit_msgs/msg/CollisionObject '{
          header: {frame_id: "world"}, 
          id: "yolo_box_001", 
          operation: [1]
        }'
*/