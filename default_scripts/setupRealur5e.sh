#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Use ROBOT_IP environment variable if set, otherwise default to 192.168.0.100
ROBOT_IP="${ROBOT_IP:-192.168.0.100}"

# gnome-terminal -t "DriverServer" -- bash -c "cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false; exec bash"
gnome-terminal -t "DriverServer" -- bash -c "cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=${ROBOT_IP} initial_joint_controller:=scaled_joint_trajectory_controller use_fake_hardware:=false launch_rviz:=false description_package:=robot_description description_file:=ur5e_with_camera.xacro; exec bash"

sleep 10

# gnome-terminal -t "MoveitServer" -- bash -c "cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true; exec bash"
gnome-terminal -t "MoveitServer" -- bash -c "cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=${ROBOT_IP} ur_type:=ur5e launch_rviz:=true use_fake_hardware:=false description_package:=robot_description description_file:=ur5e_with_camera.xacro; exec bash"
