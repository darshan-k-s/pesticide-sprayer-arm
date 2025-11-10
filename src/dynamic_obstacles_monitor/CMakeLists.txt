cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacles_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# --- dependencies ---
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)


# --- targets ---
add_executable(dynamic_obstacle_control src/dynamic_obstacle_control.cpp)
ament_target_dependencies(dynamic_obstacle_control
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
)


install(TARGETS
dynamic_obstacle_control
DESTINATION lib/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp moveit_msgs moveit_ros_planning_interface)

ament_package()
