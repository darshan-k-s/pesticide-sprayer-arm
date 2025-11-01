#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

// For Orientation constrains
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>


// For enabling the constraint path planning, edit the ompl_planning.yaml file to include:
//  projection_evaluator: joints(shoulder_pan_joint,shoulder_lift_joint)
//  enforce_constrained_state_space: true

// For more info, see https://moveit.picknik.ai/humble/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html?highlight=box_constraint%20weight

// Thanks to Jasper for testing and confirmation of this node

// ROS
#include <rclcpp/rclcpp.hpp>

// C++
#include <string>
#include <vector>

// Msgs
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
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

auto generatePoseMsg(float x,float y, float z,float qx,float qy,float qz,float qw) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI_4;
}


static moveit_msgs::msg::Constraints makeFacingDownConstraint(
    const moveit::planning_interface::MoveGroupInterface& mgi,
    double tol_rad = 10.0 * M_PI / 180.0 /* 10 degrees */){
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = mgi.getPlanningFrame();          // usually base_link/world
  oc.link_name       = mgi.getEndEffectorLink();        // e.g., "tool0" (verify in RViz)
  oc.weight          = 1.0;

  // "Facing downward" — end-effector Z axis toward -Z in the planning frame:
  tf2::Quaternion q; q.setRPY(0.0, M_PI, 0.0);
  oc.orientation = tf2::toMsg(q);

  // Reasonable tolerances; too tight will often fail planning
  oc.absolute_x_axis_tolerance = tol_rad;
  oc.absolute_y_axis_tolerance = tol_rad;
  oc.absolute_z_axis_tolerance = M_PI;

  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.orientation_constraints.push_back(oc);
  return path_constraints;
}


namespace rvt = rviz_visual_tools;


int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("moveit_scene_home_ur", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Declare/receive target pose parameters (defaults: 0.3, 0.3, 0.4)
  double target_x = 0.3, target_y = 0.3, target_z = 0.4;
  (void)node->get_parameter("target_x", target_x);
  (void)node->get_parameter("target_y", target_y);
  (void)node->get_parameter("target_z", target_z);


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  move_group_interface.setPlanningTime(10.0);

  std::string frame_id = move_group_interface.getPlanningFrame();

  // Testing to see if joint names are right
  std::cout << "These are the avalible joints: " << std::endl;
  auto jointNames = move_group_interface.getJointNames();
  for (std::string i: jointNames) {
    std::cout << i << std::endl;
  }


  //size, pos, frame, id
  // auto collision_object = generateCollisionObject( 0.08, 0.6, 0.57, 0.5, 0.2, 0.2, frame_id, "box");
  auto col_object_table = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, -0.03, frame_id, "table");
  auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.45, 0.5, frame_id, "backWall");
  auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.45, 0.25, 0.5, frame_id, "sideWall");

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //  planning_scene_interface.applyCollisionObject(collision_object);   //////////////////////////////////////////////////////////
  planning_scene_interface.applyCollisionObject(col_object_table);
  planning_scene_interface.applyCollisionObject(col_object_backWall);
  planning_scene_interface.applyCollisionObject(col_object_sideWall);
  

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_scene_home_ur");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  
  // Create visualisation tool to use with rviz
  std::cout << "Create visualisation tool to use with rviz" << std::endl;
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };

  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  ////////////////////////////////////////////////////////////////////////////////

  //  Choice of planner
  move_group_interface.setPlannerId("TRRTkConfigDefault");    // Potentially change this for better or faster results (See avalible planners in ompl_planning.yaml)

  // Make sure you’re in the same frame you used for the constraint
  move_group_interface.setPoseReferenceFrame(move_group_interface.getPlanningFrame());
  // (Important) Ensure the start state is exactly what the robot thinks it is
  move_group_interface.setStartStateToCurrentState();
  // Slow down to avoid edge cases during time parameterization
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  
  // Orientation constraint. KNOW WHEN TO UNCOMMENT
  // auto path_constraints = makeFacingDownConstraint(move_group_interface);
  // move_group_interface.setPathConstraints(path_constraints);

  // (Make sure target pose orientation matches “down”; you already use 0,1,0,0)
  auto target_pose = generatePoseMsg(target_x, target_y, target_z, 0.0, 1.0, 0.0, 0.0);
  move_group_interface.setPoseTarget(target_pose);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan planMessage;
  auto success = static_cast<bool>(move_group_interface.plan(planMessage));
    
  // Execute the plan
  if (success) {
    move_group_interface.execute(planMessage);
  } else {
    RCLCPP_ERROR(logger, "Planning failed! First");
  }

  // AFTER execution (or after a failed attempt), always clear:
  move_group_interface.clearPathConstraints();
  move_group_interface.setStartStateToCurrentState();




  /*

  ////////////////////////////////////// BOX CONSTRAINT ////////////////////////////////////////////////////////////////////

  
    // Let's try the simple box constraints first!
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
  box_constraint.link_name = move_group_interface.getEndEffectorLink();
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.4, 0.8, 0.7 };
  box_constraint.constraint_region.primitives.emplace_back(box);


  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = target_pose.position.x;
  box_pose.position.y = 0.45;
  box_pose.position.z = 0.45;
  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0; // Weird this doesnt effect anything 

  // Visualize the box constraint
  Eigen::Vector3d box_point_1(box_pose.position.x - box.dimensions[0] / 2, box_pose.position.y - box.dimensions[1] / 2,
                              box_pose.position.z - box.dimensions[2] / 2);
  Eigen::Vector3d box_point_2(box_pose.position.x + box.dimensions[0] / 2, box_pose.position.y + box.dimensions[1] / 2,
                              box_pose.position.z + box.dimensions[2] / 2);
  moveit_visual_tools.publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
  moveit_visual_tools.trigger();

  
  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);
  move_group_interface.setPathConstraints(box_constraints);
  
  */


  moveit_visual_tools.trigger();
  move_group_interface.clearPathConstraints();

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
