#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

gnome-terminal -t "DriverServer" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false description_package:=robot_description description_file:=ur5e_with_camera.xacro; exec bash'"

sleep 10

gnome-terminal -t "MoveitServer" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true description_package:=robot_description description_file:=ur5e_with_camera.xacro; exec bash'"





