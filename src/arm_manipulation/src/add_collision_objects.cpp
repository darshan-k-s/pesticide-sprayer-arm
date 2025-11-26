#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std::chrono_literals;

auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, 
                              std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0; 
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("add_collision_objects");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  move_group_interface.setPlanningTime(10.0);

  std::string frame_id = move_group_interface.getPlanningFrame();

  std::cout << "\n========================================" << std::endl;
  std::cout << "Adding collision objects to the scene..." << std::endl;
  std::cout << "========================================\n" << std::endl;

  // Create collision objects
  auto col_object_table = generateCollisionObject(2.4, 1.2, 0.04, 0.85, 0.25, -0.03, frame_id, "table");
  auto col_object_backWall = generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.25, 0.5, frame_id, "backWall");
  auto col_object_sideWall = generateCollisionObject(0.04, 1.2, 1.0, -0.25, 0.25, 0.5, frame_id, "sideWall");
  // Trash bin: 0.3m x 0.3m x 0.05m, height 0.05m (center at z=0.025m, top at z=0.05m)
  auto col_object_trashBin = generateCollisionObject(0.3, 0.3, 0.05, 0.10, 0.5, 0.025, frame_id, "trashBin");

  // Add the collision objects to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(col_object_table);
  planning_scene_interface.applyCollisionObject(col_object_backWall);
  planning_scene_interface.applyCollisionObject(col_object_sideWall);
  planning_scene_interface.applyCollisionObject(col_object_trashBin);
  
  std::cout << "✓ Added the following collision objects:" << std::endl;
  std::cout << "  - table (2.4 x 1.2 x 0.04)" << std::endl;
  std::cout << "  - backWall (2.4 x 0.04 x 1.0)" << std::endl;
  std::cout << "  - sideWall (0.04 x 1.2 x 1.0)" << std::endl;
  std::cout << "  - trashBin (0.3 x 0.3 x 0.05), height=0.05m, center at (0.10, 0.5, 0.025)" << std::endl;
  std::cout << "\nScene setup complete!" << std::endl;
  std::cout << "Collision objects have been added to the scene and will persist." << std::endl;
  
  // Collision objects will persist in the planning scene even after node shutdown
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Small delay to ensure objects are added
  
  rclcpp::shutdown();
  return 0;
}

