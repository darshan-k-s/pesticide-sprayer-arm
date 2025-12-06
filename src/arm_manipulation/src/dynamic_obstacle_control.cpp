#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

/**
 * @brief Node that subscribes to CollisionObject messages and applies them to the MoveIt Planning Scene.
 * This node acts as a bridge between an external perception system (like YOLO)
 * and the MoveIt planning environment. It uses the PlanningSceneInterface,
 * which internally handles the communication with the move_group node to update
 * the world model, ensuring obstacles are considered during motion planning.
 */
class PlanningSceneBridge : public rclcpp::Node
{
public:
    PlanningSceneBridge() : Node("planning_scene_bridge")
    {
        // Initialize Planning Scene Interface (non-blocking constructor)
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt PlanningSceneInterface...");
        RCLCPP_INFO(this->get_logger(), "PlanningSceneInterface object created (will connect on first use).");

        // Subscribe to 'obsFromImg' topic containing CollisionObject messages
        obstacle_sub_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
            "obsFromImg", 10, std::bind(&PlanningSceneBridge::obstacle_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to 'obsFromImg' and ready to update the planning scene.");
        RCLCPP_INFO(this->get_logger(), "Operation field keys: 0=ADD, 1=REMOVE");
        RCLCPP_INFO(this->get_logger(), "Node ready. Waiting for collision objects on '/obsFromImg' topic...");
    }

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr obstacle_sub_;

    void obstacle_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
    {
        std::string operation_str;

        switch (msg->operation) {
            case moveit_msgs::msg::CollisionObject::ADD:
                operation_str = "ADDING";
                break;
            case moveit_msgs::msg::CollisionObject::REMOVE:
                operation_str = "REMOVING";
                break;
            default:
                operation_str = "UNKNOWN OPERATION FOR";
        }

        RCLCPP_INFO(this->get_logger(),
            "Received object '%s' with operation: %s (%d) in frame '%s'.",
            msg->id.c_str(), operation_str.c_str(), msg->operation, msg->header.frame_id.c_str());

        std::vector<moveit_msgs::msg::CollisionObject> objects_to_apply;
        objects_to_apply.push_back(*msg);

        try {
            planning_scene_interface_.applyCollisionObjects(objects_to_apply);
            RCLCPP_DEBUG(this->get_logger(), "Successfully applied scene update for object '%s'.", msg->id.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to apply collision object '%s': %s", msg->id.c_str(), e.what());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanningSceneBridge>());
    rclcpp::shutdown();
    return 0;
}


