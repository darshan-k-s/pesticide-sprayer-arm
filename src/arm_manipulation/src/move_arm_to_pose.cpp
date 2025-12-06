/**
 * Use Pilz LIN planner for linear motion
 */
#include <memory>
#include <string>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
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
   { "shoulder_pan_joint",  0,              M_PI,         M_PI },           // 0° ± 180° = -180° to 180°
                // { "shoulder_lift_joint", -M_PI / 4,      M_PI / 4,   M_PI / 4 },   // -45° ± 45° (-90° to 0°)
   { "shoulder_lift_joint", -M_PI / 2,      M_PI / 2,     M_PI / 2 },      // -90° ± 90° -> [-180°, 0°]
   { "wrist_1_joint",       -M_PI * 105/180, M_PI,        M_PI },          // -105° ± 180°
   { "wrist_2_joint",       -M_PI / 2,      M_PI / 2,     M_PI / 2 },      // -90° ± 90°
   { "wrist_3_joint",       0,              M_PI/2,         M_PI/2 }           // 0° ± 90° (relaxed constraints to allow larger rotation range)
};
 
 int main(int argc, char* argv[])
 {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<rclcpp::Node>("move_arm_to_pose", 
                                                      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 
   double target_x , target_y , target_z ;
  bool use_constraints = true;
  double max_velocity_scaling = 0.15;  // Default velocity scaling factor (15% of max velocity)
  double max_acceleration_scaling = 0.15;  // Default acceleration scaling factor (15% of max acceleration)
  double planning_time = 60.0;  // Default planning time (60 seconds)
  int num_planning_attempts = 200;  // Default number of planning attempts (200)
  
  // Since automatically_declare_parameters_from_overrides(true) is set,
  // parameters passed from launch file will be automatically declared
  // Get parameters directly, use default values if not present
  node->get_parameter("target_x", target_x);
  node->get_parameter("target_y", target_y);
  node->get_parameter("target_z", target_z);
  // Switch parameter for enabling joint path constraints (true=enable constraints, false=no constraints)
  // Can be disabled via use_constraints:=false in launch/command line
  if (!node->get_parameter("use_constraints", use_constraints)) {
    use_constraints = true;  // Default value
  }
  // Speed limiting parameters (0.0-1.0, percentage of max velocity/acceleration)
  if (!node->get_parameter("max_velocity_scaling", max_velocity_scaling)) {
    max_velocity_scaling = 0.15;  // Default: 15% of max velocity
  }
  if (!node->get_parameter("max_acceleration_scaling", max_acceleration_scaling)) {
    max_acceleration_scaling = 0.15;  // Default: 15% of max acceleration
  }
  // Path planning parameters
  if (!node->get_parameter("planning_time", planning_time)) {
    planning_time = 60.0;  // Default: 60 seconds
  }
  if (!node->get_parameter("num_planning_attempts", num_planning_attempts)) {
    num_planning_attempts = 200;  // Default: 200 attempts
  }
  
  // Output constraint status and speed settings
  std::cout << "\n=== Moving to (" << target_x << ", " << target_y << ", " << target_z << ") ===" << std::endl;
  std::cout << "Joint constraints: " << (use_constraints ? "ENABLED" : "DISABLED") << std::endl;
  std::cout << "Speed settings: velocity=" << (max_velocity_scaling * 100) << "%, acceleration=" << (max_acceleration_scaling * 100) << "%" << std::endl;
  std::cout << "Planning settings: time=" << planning_time << "s, attempts=" << num_planning_attempts << std::endl;
 
   using moveit::planning_interface::MoveGroupInterface;
   auto move_group = MoveGroupInterface(node, "ur_manipulator");
   
   // Set end effector link to tool_point (topmost point of end effector)
   // This way target position z directly corresponds to the topmost point of end effector
   move_group.setEndEffectorLink("tool_point");
   
   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(node);
   auto spinner = std::thread([&executor]() { executor.spin(); });
   
  // Configure planning parameters (increased for better success rate)
  move_group.setPlanningTime(planning_time);  // Planning time (default 60 seconds)
  move_group.setNumPlanningAttempts(num_planning_attempts);  // Number of planning attempts (default 200)
  // Note: GoalTolerance will be set later, along with position and orientation tolerances
  // Set speed limits (using parameterized values)
  move_group.setMaxVelocityScalingFactor(max_velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(max_acceleration_scaling);
 
  // Get current pose
  auto current_pose = move_group.getCurrentPose().pose;
  std::cout << "Current position: (" << current_pose.position.x << ", " 
            << current_pose.position.y << ", " << current_pose.position.z << ")" << std::endl;

  // Print current joint values
  auto joint_names = move_group.getJointNames();
  auto joint_values = move_group.getCurrentJointValues();
  std::cout << "\nCurrent joint values:" << std::endl;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    std::cout << "  " << joint_names[i] << ": " << joint_values[i] 
              << " rad (" << (joint_values[i] * 180.0 / M_PI) << " deg)" << std::endl;
  }
  
  // Check collision detection status
  std::cout << "\n=== Collision Detection Status ===" << std::endl;
  
  // Check static collision objects
  std::cout << "\n[Static Collision Objects]" << std::endl;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Wait a bit for planning scene to sync
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  auto objects = planning_scene_interface.getObjects();
  std::cout << "Number of static collision objects in planning scene: " << objects.size() << std::endl;
  if (objects.size() > 0) {
    std::cout << "Static collision object list:" << std::endl;
    for (const auto& obj_pair : objects) {
      std::cout << "  - " << obj_pair.first << std::endl;
    }
    std::cout << " Static collision objects loaded" << std::endl;
  } else {
    std::cout << " Warning: No static collision objects in planning scene!" << std::endl;
    std::cout << "  Please ensure: ros2 launch arm_manipulation add_collision_objects_launch.py" << std::endl;
  }
  
  // Check dynamic obstacles monitor
  std::cout << "\n[Dynamic Collision Detection]" << std::endl;
  std::cout << "Dynamic obstacle monitoring topic: /obsFromImg" << std::endl;
  std::cout << "Node name: planning_scene_bridge (dynamic_obstacle_control)" << std::endl;
  std::cout << "Function: Subscribes to /obsFromImg topic, receives dynamic obstacle messages and updates planning scene" << std::endl;
  std::cout << "Status check: Please confirm dynamic_obstacles_monitor node is running" << std::endl;
  std::cout << "  - Check method: ros2 node list | grep planning_scene_bridge" << std::endl;
  std::cout << "  - Or: ros2 topic echo /obsFromImg" << std::endl;
  
  std::cout << "\nCollision Detection Mechanism:" << std::endl;
  std::cout << "  1. Static collision objects (table, walls, trash bin, etc.)" << std::endl;
  std::cout << "     - Added at startup via add_collision_objects node" << std::endl;
  std::cout << "     - Remain constant throughout operation" << std::endl;
  std::cout << "  2. Dynamic collision objects (obstacles detected from vision system)" << std::endl;
  std::cout << "     - Received in real-time via /obsFromImg topic" << std::endl;
  std::cout << "     - dynamic_obstacle_control node automatically updates planning scene" << std::endl;
  std::cout << "     - Can be dynamically added (ADD) or removed (REMOVE)" << std::endl;
  std::cout << "  3. Collision detection during path planning" << std::endl;
  std::cout << "     - MoveIt planners enable collision detection by default" << std::endl;
  std::cout << "     - Automatically avoid all static and dynamic collision objects" << std::endl;
  std::cout << "     - Collision detection performed in real-time during planning" << std::endl;
 
   // Target pose: keep current orientation
  // Note: Target position (x, y, z) is now relative to tool_point (topmost point of end effector)
  // tool_point is at the top of the end effector, so z=0 means the topmost point is at floor level
  // With this setup, if target z=0, the topmost point of end effector will be near the floor (table top)
   geometry_msgs::msg::Pose target_pose = current_pose;
   target_pose.position.x = target_x;
   target_pose.position.y = target_y;
   target_pose.position.z = target_z;
   
  // Print target pose information
  std::cout << "\nTarget Pose:" << std::endl;
  std::cout << "  Position: (" << target_pose.position.x << ", " 
            << target_pose.position.y << ", " << target_pose.position.z << ")" << std::endl;
  std::cout << "  Orientation (quaternion): (" << target_pose.orientation.x << ", " 
            << target_pose.orientation.y << ", " << target_pose.orientation.z << ", "
            << target_pose.orientation.w << ")" << std::endl;
  
  // Decide whether to add joint path constraints based on parameter switch
  if (use_constraints) {
    moveit_msgs::msg::Constraints constraints;
    
    for (const auto& config : JOINT_CONSTRAINTS) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = config.joint_name;
      jc.position = config.position;
      jc.tolerance_above = config.tolerance_above;
      jc.tolerance_below = config.tolerance_below;
      jc.weight = 1.0;
      constraints.joint_constraints.push_back(jc);
      
      // Output constraint information and verify if start point is within constraint range
      double min_val = jc.position - jc.tolerance_below;
      double max_val = jc.position + jc.tolerance_above;
      
      // Find current joint value
      double current_joint_val = 0.0;
      bool found_joint = false;
      for (size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == jc.joint_name) {
          current_joint_val = joint_values[i];
          found_joint = true;
          break;
        }
      }
      
      std::cout << "  Constraint: " << jc.joint_name 
                << " Range: [" << (min_val * 180.0 / M_PI) << "°, "
                << (max_val * 180.0 / M_PI) << "°] (center: " 
                << (jc.position * 180.0 / M_PI) << "°)";
      
      if (found_joint) {
        bool in_range = (current_joint_val >= min_val && current_joint_val <= max_val);
        std::cout << " Current: " << (current_joint_val * 180.0 / M_PI) << "°"
                  << (in_range ? " " : "  Out of range!");
        if (!in_range) {
          std::cout << "\n    Warning: Start point is not within constraint range, planning may fail!";
        }
      }
      std::cout << std::endl;
    }
    move_group.setPathConstraints(constraints);
    std::cout << "\nJoint constraints ENABLED (use_constraints=true)" << std::endl;
    std::cout << "Set " << constraints.joint_constraints.size() << " joint constraints" << std::endl;
    std::cout << "Note: Path constraints require all points along the entire path to be within constraint range, not just start and end points!" << std::endl;
  } else {
    std::cout << "Joint constraints DISABLED (use_constraints=false)" << std::endl;
  }
   
  // Set target pose with position and orientation tolerances
  // Set target pose, allowing some tolerance in position and orientation
  move_group.setPoseTarget(target_pose);
  
  // Set goal tolerances to give planner more sampling space in goal region
  // This is important for solving "Insufficient states in sampleable goal region" errors
  // Increasing tolerance makes it easier for planner to find configurations satisfying the goal
  const double goal_position_tolerance = 0.005;      // 2cm
  const double goal_orientation_tolerance = 0.1;    // ~11°
  const double goal_tolerance = 0.005;               // 2cm overall

  move_group.setGoalPositionTolerance(goal_position_tolerance);
  move_group.setGoalOrientationTolerance(goal_orientation_tolerance);
  move_group.setGoalTolerance(goal_tolerance);
  
  // Ensure start state is correct
  move_group.setStartStateToCurrentState();
  
  std::cout << "\nGoal Tolerance Settings:" << std::endl;
  std::cout << "  Position tolerance: " << goal_position_tolerance << " m" << std::endl;
  std::cout << "  Orientation tolerance: " << goal_orientation_tolerance << " rad (" 
            << (goal_orientation_tolerance * 180.0 / M_PI) << " deg)" << std::endl;
  std::cout << "  Overall tolerance: " << goal_tolerance << " m" << std::endl;
  
  // Define multiple alternative planners (sorted by priority)
  struct PlannerConfig {
    std::string name;
    std::string description;
  };
  
  std::vector<PlannerConfig> planners = {
    {"LIN", "Pilz LIN planner (linear motion)"},
    {"RRTConnectkConfigDefault", "RRTConnect (bidirectional RRT)"},
    {"PRMstarkConfigDefault", "PRM* (probabilistic roadmap, constraint-friendly)"},
    {"BITstarkConfigDefault", "BIT* (batch informed tree, optimal paths)"},
    {"KPIECEkConfigDefault", "KPIECE (grid-based, constraint-friendly)"},
    {"BKPIECEkConfigDefault", "BKPIECE (bidirectional KPIECE)"},
    {"ESTkConfigDefault", "EST (expansive space tree, constraint-friendly)"},
    {"SBLkConfigDefault", "SBL (single-query bidirectional lazy)"}
  };
  
   moveit::planning_interface::MoveGroupInterface::Plan plan;
   bool success = false;
  std::string successful_planner;
  
  // First try all planners (with constraints if enabled)
  std::cout << "\n=== Attempting Path Planning ===" << std::endl;
  for (const auto& planner_cfg : planners) {
    std::cout << "\nTrying planner: " << planner_cfg.description << " (" << planner_cfg.name << ")..." << std::endl;
    move_group.setPlannerId(planner_cfg.name);
    
    auto result = move_group.plan(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      std::cout << " " << planner_cfg.name << " planning succeeded!" << std::endl;
      std::cout << "  Planned path contains " << plan.trajectory_.joint_trajectory.points.size() << " waypoints" << std::endl;
      
      // Try to execute (MoveIt will automatically validate path before execution)
      std::cout << "Executing planned path..." << std::endl;
      auto exec_result = move_group.execute(plan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        success = true;
        successful_planner = planner_cfg.name;
        std::cout << " Execution succeeded!" << std::endl;
        break;
      } else {
        std::cout << " Execution failed (error code: " << exec_result.val << ")" << std::endl;
        if (exec_result == moveit::core::MoveItErrorCode::PLANNING_FAILED) {
          std::cout << "  Reason: Path validation failed (may be too close to collision objects)" << std::endl;
          std::cout << "  Hint: Blue box safety margin reduced to 0.5cm, if still failing, may need to adjust target position" << std::endl;
        }
        // Continue trying next planner
      }
   } else {
      std::cout << " " << planner_cfg.name << " planning failed (error code: " << result.val << ")" << std::endl;
    }
  }
  
  // If all planners failed and constraints are enabled, try without constraints
  if (!success && use_constraints) {
    std::cout << "\n=== All constrained planners failed ===" << std::endl;
    std::cout << "\nCause Analysis:" << std::endl;
    std::cout << "  Path constraints require all points along the entire path to be within constraint range," << std::endl;
    std::cout << "  not just start and end points. Even if start and end points are within constraints," << std::endl;
    std::cout << "  the path from start to end may need to pass through some intermediate states," << std::endl;
    std::cout << "  which may violate constraints, causing planning to fail." << std::endl;
    std::cout << "\nAttempting to replan with joint constraints disabled (path may not satisfy constraints)..." << std::endl;
   move_group.clearPathConstraints();
 
    // Retry a few main planners (without constraints)
    std::vector<PlannerConfig> no_constraint_planners = {
      {"RRTConnectkConfigDefault", "RRTConnect (no constraints)"},
      {"PRMstarkConfigDefault", "PRM* (no constraints)"},
      {"BITstarkConfigDefault", "BIT* (no constraints)"},
      {"KPIECEkConfigDefault", "KPIECE (no constraints)"}
    };
    
    for (const auto& planner_cfg : no_constraint_planners) {
      std::cout << "\nTrying planner: " << planner_cfg.description << " (" << planner_cfg.name << ")..." << std::endl;
      move_group.setPlannerId(planner_cfg.name);
      
      auto result = move_group.plan(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        std::cout << " " << planner_cfg.name << " planning succeeded (no constraints)!" << std::endl;
        std::cout << "  Planned path contains " << plan.trajectory_.joint_trajectory.points.size() << " waypoints" << std::endl;
        
        // Try to execute
        std::cout << "Executing planned path..." << std::endl;
        auto exec_result = move_group.execute(plan);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
          success = true;
          successful_planner = planner_cfg.name + " (no constraints)";
          std::cout << " Execution succeeded!" << std::endl;
          break;
        } else {
          std::cout << " Execution failed (error code: " << exec_result.val << ")" << std::endl;
        }
      } else {
        std::cout << " " << planner_cfg.name << " planning failed (error code: " << result.val << ")" << std::endl;
        
        // If "Insufficient states in sampleable goal region" error, provide more detailed diagnosis
        if (result == moveit::core::MoveItErrorCode::PLANNING_FAILED) {
          std::cout << "   Diagnosis: Target region may not be able to sample valid states" << std::endl;
          std::cout << "   Suggestion: Check if target position is reachable, or try different target orientation" << std::endl;
        }
      }
    }
  } else {
    move_group.clearPathConstraints();
  }

  // Output final result
   if (success) {
    std::cout << "\n Success! Planner used: " << successful_planner << std::endl;
    auto final_pose = move_group.getCurrentPose().pose;
    std::cout << "Final position: (" << final_pose.position.x << ", " 
              << final_pose.position.y << ", " << final_pose.position.z << ")" << std::endl;
  } else {
    std::cout << "\n Failed! All planners unable to find path from current position to target position." << std::endl;
    std::cout << "\nPossible solutions:" << std::endl;
    std::cout << "  1. Check if target position is within robot workspace" << std::endl;
    std::cout << "  2. Check if collision objects are blocking the path" << std::endl;
    std::cout << "  3. 'Insufficient states in sampleable goal region' error usually means:" << std::endl;
    std::cout << "     - Target position is unreachable (may be outside workspace)" << std::endl;
    std::cout << "     - Target orientation doesn't match reachable orientations" << std::endl;
    std::cout << "     - tool_point position calculation may have issues" << std::endl;
    if (use_constraints) {
      std::cout << "  4. Path constraints may be too strict, try disabling with use_constraints:=false" << std::endl;
      std::cout << "  5. Or relax joint constraint ranges (increase tolerance)" << std::endl;
      std::cout << "  6. Path constraints require entire path to satisfy constraints, not just start and end points" << std::endl;
   } else {
      std::cout << "  4. Adjust target position or use intermediate waypoints" << std::endl;
      std::cout << "  5. Check robot joint limits" << std::endl;
      std::cout << "  6. Try adjusting target pose orientation (current orientation may be unreachable)" << std::endl;
    }
   }
 
   rclcpp::shutdown();
   spinner.join();
   return 0;
 }