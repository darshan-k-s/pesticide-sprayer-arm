/**
 * Use Pilz LIN planner for linear motion
 */
#include <memory>
#include <string>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

// Joint constraint configuration (reference: ScrewDrivingBot)
struct JointConstraintConfig {
    std::string joint_name;
    double position;
    double tolerance_above;
    double tolerance_below;
};

const std::vector<JointConstraintConfig> JOINT_CONSTRAINTS = {
    { "shoulder_pan_joint",  M_PI,       M_PI / 2,   M_PI / 2 },   // 180° ± 90°
    { "shoulder_lift_joint", -M_PI / 2,  M_PI / 2,   M_PI / 2 },   // -90° ± 90°
    { "wrist_1_joint",       -M_PI / 2,  M_PI * 4/5, M_PI * 4/5 }, // -90° ± 144°
    { "wrist_2_joint",       M_PI / 2,   M_PI / 3,   M_PI / 3 },   // 90° ± 60°
    { "wrist_3_joint",       0,          M_PI / 3,   M_PI / 3 }    // 0° ± 60°
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_arm_to_pose", 
                                                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  double target_x = 0.3, target_y = 0.3, target_z = 0.4;
  node->get_parameter("target_x", target_x);
  node->get_parameter("target_y", target_y);
  node->get_parameter("target_z", target_z);

  std::cout << "\n=== Moving to (" << target_x << ", " << target_y << ", " << target_z << ") ===" << std::endl;

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  
  // Set Pilz LIN planner
  move_group.setPlannerId("LIN");
  move_group.setPlanningTime(20.0);
  move_group.setNumPlanningAttempts(15);
  move_group.setGoalTolerance(0.001);  // 1mm precision
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // Get current pose
  auto current_pose = move_group.getCurrentPose().pose;
  std::cout << "Current position: (" << current_pose.position.x << ", " 
            << current_pose.position.y << ", " << current_pose.position.z << ")" << std::endl;

  // Target pose: keep current orientation
  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.x = target_x;
  target_pose.position.y = target_y;
  target_pose.position.z = target_z;
  
  // Set joint constraints
  moveit_msgs::msg::Constraints constraints;
  for (const auto& config : JOINT_CONSTRAINTS) {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = config.joint_name;
    jc.position = config.position;
    jc.tolerance_above = config.tolerance_above;
    jc.tolerance_below = config.tolerance_below;
    jc.weight = 1.0;
    constraints.joint_constraints.push_back(jc);
  }
  move_group.setPathConstraints(constraints);
  std::cout << "Joint constraints set" << std::endl;
  
  // Set target pose
  move_group.setPoseTarget(target_pose);
  
  // Plan and execute
  std::cout << "Using Pilz LIN planner..." << std::endl;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;
  
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << "✓ LIN planning succeeded, executing..." << std::endl;
    success = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  } else {
    // Fallback to RRTConnect
    std::cout << "LIN planning failed, trying RRTConnect..." << std::endl;
    move_group.setPlannerId("RRTConnectkConfigDefault");
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      std::cout << "✓ RRTConnect planning succeeded, executing..." << std::endl;
      success = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
  }
  
  move_group.clearPathConstraints();

  if (success) {
    std::cout << "\n✓ Done!" << std::endl;
  } else {
    std::cout << "\n✗ Failed!" << std::endl;
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
