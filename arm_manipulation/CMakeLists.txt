cmake_minimum_required(VERSION 3.8)
project(arm_manipulation)

# --- build settings ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# --- dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# --- targets ---
add_executable(moveit_scene_home_ur src/moveit_scene_home_ur.cpp)
target_compile_features(moveit_scene_home_ur PUBLIC cxx_std_17)
ament_target_dependencies(moveit_scene_home_ur
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  moveit_msgs
)

# --- install ---
install(TARGETS moveit_scene_home_ur
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(moveit_go_to_pose_ur src/moveit_go_to_pose_ur.cpp)
target_compile_features(moveit_go_to_pose_ur PUBLIC cxx_std_17)
ament_target_dependencies(moveit_go_to_pose_ur
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)
install(TARGETS moveit_go_to_pose_ur DESTINATION lib/${PROJECT_NAME})


# Install resource folders if present (avoids CMake errors when absent)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
endif()

# --- testing (optional) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
