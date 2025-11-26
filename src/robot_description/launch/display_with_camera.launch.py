from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ⚠️ These are verified calibration parameters, ensure you understand them before modifying
    # Position (meters)
    x, y, z = '1.29163', '0.1146956', '0.765489'
    
    # Quaternion (verified values, corresponding to RPY)
    # RPY: roll=0.0372615, pitch=-0.815369, yaw=-3.12141
    qx, qy, qz, qw = '-0.396568', '-0.0131018', '0.91791', '-0.00187618'
    
    return LaunchDescription([
        # Transform from base_link to camera_link (hand-eye calibration)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=[x, y, z, qx, qy, qz, qw, 'base_link', 'camera_link']
        ),
        # Transform from camera_link to camera_color_optical_frame
        # RealSense standard optical frame rotation
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_frame_broadcaster',
            arguments=['0', '0', '0', '0.5', '-0.5', '0.5', '0.5',
                      'camera_link', 'camera_color_optical_frame']
        )
    ])
