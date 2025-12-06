#!/usr/bin/env python3

"""
Launch file for moving the arm to a target pose

Usage:
  ros2 launch arm_manipulation move_arm_to_pose_launch.py
  ros2 launch arm_manipulation move_arm_to_pose_launch.py x:=0.5 y:=0.2 z:=0.6
  ros2 launch arm_manipulation move_arm_to_pose_launch.py x:=0.5 y:=0.2 z:=0.6 use_constraints:=false
  ros2 launch arm_manipulation move_arm_to_pose_launch.py x:=0.5 y:=0.2 z:=0.6 planning_time:=180 num_planning_attempts:=300
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
    use_constraints_arg = DeclareLaunchArgument(
        'use_constraints', 
        default_value='true', 
        description='Enable joint path constraints (true/false)'
    )
    
    # Speed limiting arguments (0.0-1.0, percentage of max velocity/acceleration)
    max_velocity_scaling_arg = DeclareLaunchArgument(
        'max_velocity_scaling',
        default_value='0.15',
        description='Maximum velocity scaling factor (0.0-1.0, default: 0.15 = 15% max speed)'
    )
    max_acceleration_scaling_arg = DeclareLaunchArgument(
        'max_acceleration_scaling',
        default_value='0.15',
        description='Maximum acceleration scaling factor (0.0-1.0, default: 0.15 = 15% max acceleration)'
    )
    
    # Planning parameters
    planning_time_arg = DeclareLaunchArgument(
        'planning_time',
        default_value='60.0',
        description='Maximum planning time in seconds (default: 60.0 seconds)'
    )
    num_planning_attempts_arg = DeclareLaunchArgument(
        'num_planning_attempts',
        default_value='200',
        description='Maximum number of planning attempts (default: 200)'
    )
    
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
            'use_constraints': ParameterValue(LaunchConfiguration('use_constraints'), value_type=bool),
            'max_velocity_scaling': ParameterValue(LaunchConfiguration('max_velocity_scaling'), value_type=float),
            'max_acceleration_scaling': ParameterValue(LaunchConfiguration('max_acceleration_scaling'), value_type=float),
            'planning_time': ParameterValue(LaunchConfiguration('planning_time'), value_type=float),
            'num_planning_attempts': ParameterValue(LaunchConfiguration('num_planning_attempts'), value_type=int),
        }],
    )

    return launch.LaunchDescription([
        x_arg, 
        y_arg, 
        z_arg, 
        use_constraints_arg,
        max_velocity_scaling_arg,
        max_acceleration_scaling_arg,
        planning_time_arg,
        num_planning_attempts_arg,
        move_arm_node
    ])

