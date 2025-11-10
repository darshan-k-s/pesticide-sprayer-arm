#!/usr/bin/env python3

"""
Launch file for moving the arm to a target pose

Usage:
  ros2 launch arm_manipulation move_arm_to_pose_launch.py
  ros2 launch arm_manipulation move_arm_to_pose_launch.py x:=0.5 y:=0.2 z:=0.6
"""

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare CLI args for target pose
    x_arg = DeclareLaunchArgument('x', default_value='0.3', description='Target X in metres')
    y_arg = DeclareLaunchArgument('y', default_value='0.3', description='Target Y in metres')
    z_arg = DeclareLaunchArgument('z', default_value='0.4', description='Target Z in metres')
    
    # Node to move the arm
    # Note: Environment variables should be set in the shell before launching
    # or use a wrapper script if Conda library conflicts exist
    move_arm_node = Node(
        package="arm_manipulation",
        executable="move_arm_to_pose",
        name="move_arm_to_pose",
        output="screen",
        parameters=[{
            # ensure float typing even though LaunchConfiguration returns strings
            'target_x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'target_y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'target_z': ParameterValue(LaunchConfiguration('z'), value_type=float),
        }],
    )

    return launch.LaunchDescription([x_arg, y_arg, z_arg, move_arm_node])

