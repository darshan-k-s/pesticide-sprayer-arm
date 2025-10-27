#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <string>
#include <vector>
#include <algorithm>

using MoveGroup = moveit_msgs::action::MoveGroup;




// ros2 run arm_manipulation moveit_go_to_pose_ur -- -0.212 -1.488 0



static bool parse_cli_pose_after_dashdash(
  int argc, char** argv,
  double& x, double& y, double& z,
  bool& use_rpy, double& r, double& p, double& yw)
{
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--") {
      std::vector<double> v;
      for (int j = i + 1; j < argc; ++j) {
        try { v.emplace_back(std::stod(argv[j])); } catch (...) { return false; }
      }
      if (v.size() == 3) { x=v[0]; y=v[1]; z=v[2]; use_rpy=true; r=M_PI; p=0.0; yw=0.0; return true; }
      if (v.size() == 6) { x=v[0]; y=v[1]; z=v[2]; use_rpy=true; r=v[3]; p=v[4]; yw=v[5]; return true; }
      return false;
    }
  }
  return false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_go_to_pose_ur");

  // --- Params (override with --ros-args -p ... or CLI "-- x y z [r p y]") ---
  const std::string planning_group = node->declare_parameter<std::string>("planning_group", "ur_manipulator");
  const std::string ee_link        = node->declare_parameter<std::string>("end_effector_link", "tool0");
  const std::string pose_frame     = node->declare_parameter<std::string>("pose_frame", "base_link");

  double x = node->declare_parameter<double>("x", 0.45);
  double y = node->declare_parameter<double>("y", 0.10);
  double z = node->declare_parameter<double>("z", 0.18);

  bool use_rpy = node->declare_parameter<bool>("use_rpy", true);
  double roll  = node->declare_parameter<double>("roll", M_PI);   // tool-down
  double pitch = node->declare_parameter<double>("pitch", 0.0);
  double yaw   = node->declare_parameter<double>("yaw", 0.0);

  // tolerances (EE must reach within these)
//   const double pos_tol = node->declare_parameter<double>("pos_tol", 0.01);      // meters (sphere radius)
//   const double oc_x    = node->declare_parameter<double>("oc_tol_x", 0.15);     // radians
//   const double oc_y    = node->declare_parameter<double>("oc_tol_y", 0.15);
//   const double oc_z    = node->declare_parameter<double>("oc_tol_z", 3.14);



  const double pos_tol = node->declare_parameter<double>("pos_tol", 0.05);      // 5 cm sphere: much easier to satisfy
  const double oc_x    = node->declare_parameter<double>("oc_tol_x", 0.5);      // ~29°
  const double oc_y    = node->declare_parameter<double>("oc_tol_y", 0.5);
  const double oc_z    = node->declare_parameter<double>("oc_tol_z", 3.14);     // free yaw



  // scaling + time
  const double vel_scale = std::clamp(node->declare_parameter<double>("vel_scale", 0.2), 0.0, 1.0);
  const double acc_scale = std::clamp(node->declare_parameter<double>("acc_scale", 0.2), 0.0, 1.0);
//   const double planning_time = node->declare_parameter<double>("planning_time", 10.0);
  const double planning_time = node->declare_parameter<double>("planning_time", 15.0);


  // Optional CLI override after "--"
  (void)parse_cli_pose_after_dashdash(argc, argv, x, y, z, use_rpy, roll, pitch, yaw);

  // Compose desired orientation
  geometry_msgs::msg::Quaternion q_msg;
  if (use_rpy) {
    tf2::Quaternion q; q.setRPY(roll, pitch, yaw);
    q_msg = tf2::toMsg(q);
  } else {
    q_msg.w = 1.0;
  }

  // --- Build goal constraints (no local robot model needed) ---
  moveit_msgs::msg::Constraints goal_constraints;

  // Position constraint as a small sphere region around the target
  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = pose_frame;
  pc.link_name = ee_link;
  pc.target_point_offset.x = 0.0;
  pc.target_point_offset.y = 0.0;
  pc.target_point_offset.z = 0.0;

  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = sphere.SPHERE;
  sphere.dimensions.resize(1);
  sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = std::max(1e-4, pos_tol);

  geometry_msgs::msg::Pose region_pose;
  region_pose.orientation.w = 1.0;
  region_pose.position.x = x;
  region_pose.position.y = y;
  region_pose.position.z = z;

  pc.constraint_region.primitives.push_back(sphere);
  pc.constraint_region.primitive_poses.push_back(region_pose);
  pc.weight = 1.0;

  goal_constraints.position_constraints.push_back(pc);

  // Orientation constraint
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = pose_frame;
  oc.link_name = ee_link;
  oc.orientation = q_msg;
  oc.absolute_x_axis_tolerance = std::max(1e-4, oc_x);
  oc.absolute_y_axis_tolerance = std::max(1e-4, oc_y);
  oc.absolute_z_axis_tolerance = std::max(1e-4, oc_z);
  oc.weight = 1.0;

  goal_constraints.orientation_constraints.push_back(oc);

  // --- Build MoveGroup goal ---
  MoveGroup::Goal goal;
  goal.request.group_name = planning_group;
  //goal.request.pipeline_id = "ompl";
  goal.request.allowed_planning_time = planning_time;
  goal.request.num_planning_attempts = 1;
  goal.request.max_velocity_scaling_factor = vel_scale;
  goal.request.max_acceleration_scaling_factor = acc_scale;

  // Workspace (avoids warning; adjust as needed)
  goal.request.workspace_parameters.header.frame_id = pose_frame;
  goal.request.workspace_parameters.min_corner.x = -1.5;
  goal.request.workspace_parameters.min_corner.y = -1.5;
  goal.request.workspace_parameters.min_corner.z =  0.0;
  goal.request.workspace_parameters.max_corner.x =  1.5;
  goal.request.workspace_parameters.max_corner.y =  1.5;
  goal.request.workspace_parameters.max_corner.z =  1.5;



  

  // Use current state as start
  goal.request.start_state.is_diff = true;

  // Pose goal
  goal.request.goal_constraints.clear();
  goal.request.goal_constraints.push_back(goal_constraints);

  // Execute, not plan-only
  goal.planning_options.plan_only = false;
  goal.planning_options.look_around = false;
  goal.planning_options.replan = false;
  goal.planning_options.replan_attempts = 0;
  goal.planning_options.planning_scene_diff.is_diff = true;

  // --- Action client to /move_action ---
  auto client = rclcpp_action::create_client<MoveGroup>(node, "move_action");
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "MoveGroup action server 'move_action' not available.");
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(node->get_logger(),
              "Requesting plan to (%0.3f, %0.3f, %0.3f) in %s, ee=%s (RPY roll=%0.3f pitch=%0.3f yaw=%0.3f)",
              x, y, z, pose_frame.c_str(), ee_link.c_str(), roll, pitch, yaw);

  rclcpp_action::Client<MoveGroup>::SendGoalOptions opts;
  auto gh_future = client->async_send_goal(goal, opts);

  // Spin until goal accepted
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  if (exec.spin_until_future_complete(gh_future, std::chrono::seconds(15)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Send goal timed out.");
    rclcpp::shutdown();
    return 3;
  }

  auto gh = gh_future.get();
  if (!gh) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server.");
    rclcpp::shutdown();
    return 3;
  }

  // Wait for result
  auto res_future = client->async_get_result(gh);
  if (exec.spin_until_future_complete(res_future, std::chrono::seconds(120)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Result wait timed out.");
    rclcpp::shutdown();
    return 4;
  }

  auto res = res_future.get();
  const int32_t code = res.result->error_code.val;
  const bool ok = (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "MoveGroup result: %s (code=%d)", ok ? "SUCCESS" : "FAIL", code);

  rclcpp::shutdown();
  return ok ? 0 : 5;
}
