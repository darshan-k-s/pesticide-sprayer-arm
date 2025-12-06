#!/usr/bin/env python3

"""
Launch file for adding collision objects to the scene

Usage:
  ros2 launch arm_manipulation add_collision_objects_launch.py
"""

import launch
from launch_ros.actions import Node

def generate_launch_description():
    # Node to add collision objects
    # Note: Environment variables should be set in the shell before launching
    # or use a wrapper script if Conda library conflicts exist
    add_objects_node = Node(
        package="arm_manipulation",
        executable="add_collision_objects",
        name="add_collision_objects",
        output="screen",
    )

    return launch.LaunchDescription([add_objects_node])

