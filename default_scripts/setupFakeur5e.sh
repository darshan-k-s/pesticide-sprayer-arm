#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

gnome-terminal -t "DriverServer" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=scaled_joint_trajectory_controller use_fake_hardware:=true launch_rviz:=false description_package:=robot_description description_file:=ur5e_with_camera.xacro; exec bash'"

sleep 5

gnome-terminal -t "MoveitServer" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_fake_hardware:=true description_package:=robot_description description_file:=ur5e_with_camera.xacro; exec bash'"