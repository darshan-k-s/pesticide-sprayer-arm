from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    
    # Kill any existing robot_state_publisher and rviz2 processes before starting
    kill_old_processes = ExecuteProcess(
        cmd=['sh', '-c', 'killall -9 robot_state_publisher rviz2 joint_state_publisher_gui || true'],
        output='screen'
    )
    
    # Use Command to properly process xacro with arguments
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "ur5e_with_camera.xacro"]),
            " ",
            "name:=ur5e",
            " ",
            "ur_type:=ur5e",
            " ",
            "use_fake_hardware:=true",
            " ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Joint state publisher GUI for manual joint control
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Use UR package's default RViz config
    ur_pkg_share = get_package_share_directory('ur_description')
    ur_rviz_config = os.path.join(ur_pkg_share, 'rviz', 'view_robot.rviz')
    rviz_args = ['-d', ur_rviz_config] if os.path.exists(ur_rviz_config) else []
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen'
    )

    # Start nodes only after kill process finishes
    delay_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=kill_old_processes,
            on_exit=[rsp, jsp_gui, rviz2]
        )
    )

    return LaunchDescription([kill_old_processes, delay_start_handler])



