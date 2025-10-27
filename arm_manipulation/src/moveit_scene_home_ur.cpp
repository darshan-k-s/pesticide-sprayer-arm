#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>  // for std::find
#include <chrono>     // for std::chrono
#include <vector>
#include <string>

static moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id, const std::string& frame,
  double sx, double sy, double sz, double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = frame;
  obj.id = id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions.resize(3);
  prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = sx;
  prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = sy;
  prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = sz;

  geometry_msgs::msg::Pose p;
  p.orientation.w = 1.0;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(p);
  obj.operation = obj.ADD;
  return obj;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "moveit_scene_home_ur");

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mgi(node, "ur_manipulator");
  mgi.setPlanningTime(10.0);

  const std::string frame = mgi.getPlanningFrame();  // usually "base_link"

  moveit::planning_interface::PlanningSceneInterface psi;

  // --- Planning scene (table + walls only; NO middle box) ---
  // auto box      = makeBox("box",      frame, 0.08, 0.60, 0.57,  0.50,  0.20, 0.20);  // removed
  auto table    = makeBox("table",    frame, 2.40, 1.20, 0.04,  0.85,  0.25,-0.03);
  auto backWall = makeBox("backWall", frame, 2.40, 0.04, 1.00,  0.85, -0.45, 0.50);
  auto sideWall = makeBox("sideWall", frame, 0.04, 1.20, 1.00, -0.45,  0.25, 0.50);

  psi.applyCollisionObjects({/*box,*/ table, backWall, sideWall});

  // Ensure any previously existing "box" is removed from the planning scene
  psi.removeCollisionObjects(std::vector<std::string>{"box"});

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  rclcpp::shutdown();
  return 0;
}
