from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=['1.29163', '0.0146956', '0.665489',
                      '-0.396568', '-0.0131018', '0.91791', '-0.00187618',
                      'base_link', 'camera_link']
        )
    ])
