#!/usr/bin/env python3
"""
Automation Task Launch File
Launch automation orchestrator node
"""

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """Generate launch description"""
    
    # Declare launch arguments
    min_area_arg = DeclareLaunchArgument(
        'min_area',
        default_value='0.0',
        description='Minimum leaf area threshold for detection'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.0',
        description='Leaf detection confidence threshold'
    )
    
    bias_x_arg = DeclareLaunchArgument(
        'bias_x',
        default_value='0.0',
        description='X-axis coordinate bias (meters)'
    )
    
    bias_y_arg = DeclareLaunchArgument(
        'bias_y',
        default_value='0.0',
        description='Y-axis coordinate bias (meters)'
    )
    
    bias_z_arg = DeclareLaunchArgument(
        'bias_z',
        default_value='0.1',
        description='Z-axis coordinate bias (meters)'
    )
    
    z_min_arg = DeclareLaunchArgument(
        'z_min',
        default_value='0.02',
        description='Z coordinate minimum constraint (meters)'
    )
    
    z_max_arg = DeclareLaunchArgument(
        'z_max',
        default_value='1.80',
        description='Z coordinate maximum constraint (meters)'
    )
    
    home_x_arg = DeclareLaunchArgument(
        'home_x',
        default_value='0.25',
        description='Home position X coordinate (meters)'
    )
    
    home_y_arg = DeclareLaunchArgument(
        'home_y',
        default_value='0.10',
        description='Home position Y coordinate (meters)'
    )
    
    home_z_arg = DeclareLaunchArgument(
        'home_z',
        default_value='0.55',
        description='Home position Z coordinate (meters)'
    )
    
    trash_x_arg = DeclareLaunchArgument(
        'trash_x',
        default_value='0.10',
        description='Trash bin X coordinate (meters)'
    )
    
    trash_y_arg = DeclareLaunchArgument(
        'trash_y',
        default_value='0.50',
        description='Trash bin Y coordinate (meters)'
    )
    
    trash_z_arg = DeclareLaunchArgument(
        'trash_z',
        default_value='0.20',
        description='Trash bin discard position Z coordinate (meters)'
    )
    
    # Create automation orchestrator node
    orchestrator_node = Node(
        package='task_automation',
        executable='automation_orchestrator',
        name='automation_orchestrator',
        output='screen',
        parameters=[{
            'min_area': ParameterValue(LaunchConfiguration('min_area'), value_type=float),
            'confidence': ParameterValue(LaunchConfiguration('confidence'), value_type=float),
            'bias_x': ParameterValue(LaunchConfiguration('bias_x'), value_type=float),
            'bias_y': ParameterValue(LaunchConfiguration('bias_y'), value_type=float),
            'bias_z': ParameterValue(LaunchConfiguration('bias_z'), value_type=float),
            'z_min': ParameterValue(LaunchConfiguration('z_min'), value_type=float),
            'z_max': ParameterValue(LaunchConfiguration('z_max'), value_type=float),
            'home_x': ParameterValue(LaunchConfiguration('home_x'), value_type=float),
            'home_y': ParameterValue(LaunchConfiguration('home_y'), value_type=float),
            'home_z': ParameterValue(LaunchConfiguration('home_z'), value_type=float),
            'trash_x': ParameterValue(LaunchConfiguration('trash_x'), value_type=float),
            'trash_y': ParameterValue(LaunchConfiguration('trash_y'), value_type=float),
            'trash_z': ParameterValue(LaunchConfiguration('trash_z'), value_type=float),
        }],
    )
    
    # Launch info
    info_msg = LogInfo(
        msg='Automation task orchestrator started'
    )
    
    return launch.LaunchDescription([
        min_area_arg,
        confidence_arg,
        bias_x_arg,
        bias_y_arg,
        bias_z_arg,
        z_min_arg,
        z_max_arg,
        home_x_arg,
        home_y_arg,
        home_z_arg,
        trash_x_arg,
        trash_y_arg,
        trash_z_arg,
        info_msg,
        orchestrator_node,
    ])

