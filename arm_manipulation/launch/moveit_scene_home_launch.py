
# ros2 launch arm_manipulation moveit_scene_home_launch.py x:=0.25 y:=0.10 z:=0.55

import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic
    
def generate_launch_description():
    # Declare CLI args for target pose
    x_arg = DeclareLaunchArgument('x', default_value='0.3', description='Target X in metres')
    y_arg = DeclareLaunchArgument('y', default_value='0.3', description='Target Y in metres')
    z_arg = DeclareLaunchArgument('z', default_value='0.4', description='Target Z in metres')



    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    demo_node = Node(
        package="arm_manipulation",
        executable="moveit_scene_home_ur",
        name="moveit_scene_home_ur",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {
            # ensure float typing even though LaunchConfiguration returns strings
            'target_x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'target_y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'target_z': ParameterValue(LaunchConfiguration('z'), value_type=float),
            },
        ],
    )

    return launch.LaunchDescription([x_arg, y_arg, z_arg, demo_node])
