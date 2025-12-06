from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #  These are verified calibration parameters, ensure you understand them before modifying
    # Position (meters)
    # x, y, z = '1.30317', '0.0174152', '0.675776' G2
    # x, y, z = '1.27354', '0.0326318', '0.68138' # G6
    # x, y, z = '1.29163', '0.0146956', '0.665489' # G5
    # x, y, z = '1.27677', '0.0175114', '0.673798' # G4
    x, y, z = '1.30893', '0.059849', '0.680372' # G1
    # Quaternion (verified values, corresponding to RPY)
    # RPY: roll=0.0372615, pitch=-0.815369, yaw=-3.12141
        # 1.29163 0.0146956 0.665489
    # qx, qy, qz, qw = '-0.388123', '-0.0054127', '0.92155', '-0.0087602' G2
    # qx, qy, qz, qw = '-0.397486', '-0.00834818', '0.91757', '-0.000592307' #G6
    # qx, qy, qz, qw = '-0.396568', '-0.0131018', '0.91791', '-0.00187618' #G5
    # qx, qy, qz, qw = '-0.414096', '-0.019425', '0.910018', '-0.00376407' #G4
    qx, qy, qz, qw = '-0.399127', '-0.0147756', '0.916733', '-0.0089035' #G1
    return LaunchDescription([
        # Transform from base_link to camera_link (hand-eye calibration)
        # Use new-style arguments for static_transform_publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=[
                '--x', x,
                '--y', y,
                '--z', z,
                '--qx', qx,
                '--qy', qy,
                '--qz', qz,
                '--qw', qw,
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),
        # Transform from camera_link to camera_color_optical_frame
        # RealSense standard optical frame rotation (new-style arguments)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_frame_broadcaster',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--qx', '0.5',
                '--qy', '-0.5',
                '--qz', '0.5',
                '--qw', '0.5',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
        ),
    ])
