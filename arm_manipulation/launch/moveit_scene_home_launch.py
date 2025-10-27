from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def _robot_description(prefix, use_fake, name):
    xacro = FindExecutable(name="xacro")
    urdf = PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"])
    jl   = PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"])
    kin  = PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"])
    phys = PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"])
    vis  = PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"])
    return {
        "robot_description": Command([
            xacro, " ", urdf, " ",
            "ur_type:=ur5e ",               # hardware model is ur5e
            "name:=", name, " ",            # <— configurable robot name
            "prefix:=", prefix, " ",
            "joint_limit_params:=", jl, " ",
            "kinematics_params:=", kin, " ",
            "physical_params:=",  phys, " ",
            "visual_params:=",    vis, " ",
            "use_fake_hardware:=", use_fake, " ",   # <— pass LaunchConfiguration directly
            "safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20"
        ])
    }

def _robot_description_semantic(prefix, name):
    xacro = FindExecutable(name="xacro")
    srdf = PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"])
    return {
        "robot_description_semantic": Command([
            xacro, " ", srdf, " ",
            "name:=", name, " ",              # <— same name as URDF
            "prefix:=", prefix
        ])
    }

def generate_launch_description():
    prefix      = LaunchConfiguration("prefix")
    use_fake    = LaunchConfiguration("use_fake_hardware")
    group       = LaunchConfiguration("planning_group")
    robot_name  = LaunchConfiguration("robot_name")      # NEW

    pkg_share = FindPackageShare("arm_manipulation")
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    kinematics_yaml  = PathJoinSubstitution([pkg_share, "config", "kinematics.yaml"])
    ompl_yaml        = PathJoinSubstitution([pkg_share, "config", "ompl_planning.yaml"])


    # Keep the YAML:
    ompl_yaml = PathJoinSubstitution([pkg_share, "config", "ompl_planning.yaml"])

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            _robot_description(prefix, use_fake, robot_name),
            _robot_description_semantic(prefix, robot_name),
            controllers_yaml, kinematics_yaml, ompl_yaml,
            {"planning_pipelines": {"pipeline_names": ["ompl"]},
            "default_planning_pipeline": "ompl"}
        ]
    )


    scene_home = Node(
        package="arm_manipulation",
        executable="moveit_scene_home_ur",
        output="screen",
        parameters=[
            _robot_description(prefix, use_fake, robot_name),
            _robot_description_semantic(prefix, robot_name),
            {"planning_group": group}
        ]
    )

    # Start scene after move_group is up
    delayed_scene_home = TimerAction(period=1.5, actions=[scene_home])

    return LaunchDescription([
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("planning_group", default_value="ur_manipulator"),
        DeclareLaunchArgument("robot_name", default_value="ur"),  # set to 'ur5e' if RViz loads that
        move_group,
        delayed_scene_home,
    ])
