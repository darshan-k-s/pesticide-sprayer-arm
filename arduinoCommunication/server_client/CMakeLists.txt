cmake_minimum_required(VERSION 3.8)
project(server_client)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# Generate the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LeafCommand.srv"
)

# Get the typesupport target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Build leafServerNode
add_executable(leafServerNode src/leafServer.cpp)
ament_target_dependencies(leafServerNode rclcpp)
target_link_libraries(leafServerNode "${cpp_typesupport_target}")

# Build vacuum_pump_node
add_executable(vacuum_pump_node src/vacuumPumpClient.cpp)
ament_target_dependencies(vacuum_pump_node rclcpp)
target_link_libraries(vacuum_pump_node "${cpp_typesupport_target}")

# Build leaf_spray_node
add_executable(leaf_spray_node src/sprayPumpClient.cpp)
ament_target_dependencies(leaf_spray_node rclcpp)
target_link_libraries(leaf_spray_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  leafServerNode
  vacuum_pump_node
  leaf_spray_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()