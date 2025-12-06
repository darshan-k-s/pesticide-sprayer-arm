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
        default_value='0.035',
        description='X-axis coordinate bias (meters)'
    )
    
    bias_y_arg = DeclareLaunchArgument(
        'bias_y',
        default_value='-0.05',
        description='Y-axis coordinate bias (meters)'
    )
    
    bias_z_arg = DeclareLaunchArgument(
        'bias_z',
        default_value='0.050',
        description='Z-axis coordinate bias (meters)'
    )
    
    z_min_arg = DeclareLaunchArgument(
        'z_min',
        default_value='0.040',
        description='Z coordinate minimum constraint (meters)'
    )
    
    z_max_arg = DeclareLaunchArgument(
        'z_max',
        default_value='1.0',
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
        default_value='0.30',
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
    
    arduino_action_wait_arg = DeclareLaunchArgument(
        'arduino_action_wait',
        default_value='3.0',
        description='Wait time after Arduino action (seconds)'
    )
    
    spray_height_offset_arg = DeclareLaunchArgument(
        'spray_height_offset',
        default_value='0.08',
        description='Additional height offset for spray operation (meters)'
    )
    
    use_joint_constraints_arg = DeclareLaunchArgument(
        'use_joint_constraints',
        default_value='true',
        description='Enable joint path constraints for arm movement (true/false)'
    )
    
    max_velocity_scaling_arg = DeclareLaunchArgument(
        'max_velocity_scaling',
        default_value='0.10',
        description='Maximum velocity scaling factor (0.0-1.0, default: 0.15 = 15% max speed)'
    )
    
    max_acceleration_scaling_arg = DeclareLaunchArgument(
        'max_acceleration_scaling',
        default_value='0.10',
        description='Maximum acceleration scaling factor (0.0-1.0, default: 0.15 = 15% max acceleration)'
    )
    
    unhealthy_z_threshold_arg = DeclareLaunchArgument(
        'unhealthy_z_threshold',
        default_value='0.10',
        description='Z coordinate threshold for unhealthy leaves (meters). If Z >= threshold, use original value; if Z < threshold, use unhealthy_z_min'
    )
    
    unhealthy_z_min_arg = DeclareLaunchArgument(
        'unhealthy_z_min',
        default_value='0.04',
        description='Minimum Z coordinate for unhealthy leaves when Z < threshold (meters)'
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
            'arduino_action_wait': ParameterValue(LaunchConfiguration('arduino_action_wait'), value_type=float),
            'spray_height_offset': ParameterValue(LaunchConfiguration('spray_height_offset'), value_type=float),
            'use_joint_constraints': ParameterValue(LaunchConfiguration('use_joint_constraints'), value_type=bool),
            'max_velocity_scaling': ParameterValue(LaunchConfiguration('max_velocity_scaling'), value_type=float),
            'max_acceleration_scaling': ParameterValue(LaunchConfiguration('max_acceleration_scaling'), value_type=float),
            'unhealthy_z_threshold': ParameterValue(LaunchConfiguration('unhealthy_z_threshold'), value_type=float),
            'unhealthy_z_min': ParameterValue(LaunchConfiguration('unhealthy_z_min'), value_type=float),
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
        arduino_action_wait_arg,
        spray_height_offset_arg,
        use_joint_constraints_arg,
        max_velocity_scaling_arg,
        max_acceleration_scaling_arg,
        unhealthy_z_threshold_arg,
        unhealthy_z_min_arg,
        info_msg,
        orchestrator_node,
    ])

