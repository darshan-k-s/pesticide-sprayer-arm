cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)

project(robot_description)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch rviz urdf config meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

