#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leaf Detection Server Launch File
Launches the new service-based leaf detection server
"""

from launch import LaunchDescription
from launch import conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch the leaf detection server with service interface and visualization node
    """
    
    # Declare launch arguments
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization node'
    )
    
    # Leaf detection server node
    leaf_detection_server_node = Node(
        package='detect_leaf_pkg',
        executable='leaf_detection_server',
        name='leaf_detection_server',
        output='screen'
    )
    
    # Leaf visualization node (independent node, subscribes to annotated images)
    leaf_visualization_node = Node(
        package='detect_leaf_pkg',
        executable='leaf_visualization_node',
        name='leaf_visualization_node',
        output='screen',
        condition=conditions.IfCondition(LaunchConfiguration('enable_visualization'))
    )
    
    return LaunchDescription([
        enable_visualization_arg,
        leaf_detection_server_node,
        leaf_visualization_node,
    ])

