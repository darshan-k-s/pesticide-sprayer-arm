from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Transform from base_link to camera_link (hand-eye calibration)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=['1.29163', '0.0146956', '0.665489',
                      '-0.396568', '-0.0131018', '0.91791', '-0.00187618',
                      'base_link', 'camera_link']
        ),
        # Transform from camera_link to camera_color_optical_frame
        # RealSense standard transform: rotation around X by -90°, then Z by 90°
        # Quaternion: (0.5, -0.5, 0.5, 0.5) for this rotation
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_frame_broadcaster',
            arguments=['0', '0', '0',
                      '0.5', '-0.5', '0.5', '0.5',
                      'camera_link', 'camera_color_optical_frame']
        )
    ])
