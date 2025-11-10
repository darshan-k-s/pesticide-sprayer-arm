# ROS2 URDF Workspace - UR5e

This is a basic ROS2 workspace containing URDF descriptions for the UR5e robot arm.

## Directory Structure

```
ros2_urdf_workspace/
├── src/
│   └── robot_description/          # Robot description package
│       ├── urdf/                    # URDF file directory
│       │   ├── camera.urdf.xacro           # Camera description xacro
│       │   └── ur5e_with_camera.urdf.xacro # UR5e + Camera combined xacro
│       ├── launch/                  # Launch file directory
│       │   ├── display.launch.py           # Basic robot display
│       │   └── display_with_camera.launch.py # Robot display with camera
│       ├── config/                   # Configuration file directory
│       │   └── camera_extrinsics.yaml     # Camera extrinsics configuration (hand-to-eye)
│       ├── rviz/                     # RViz configuration file directory
│       ├── CMakeLists.txt
│       └── package.xml
├── .gitignore
└── README.md
```

## Usage

### 1. Build Workspace

```bash
cd ros2_urdf_workspace
colcon build
source install/setup.bash
```

### 2. Launch Robot Visualization

**Basic Robot (without camera):**
```bash
ros2 launch robot_description display.launch.py
```

**Robot + Camera (hand-to-eye calibrated):**
```bash
ros2 launch robot_description display_with_camera.launch.py
```

This will launch:
- Robot State Publisher: Publishes robot TF transforms (including camera)
- Joint State Publisher GUI: Graphical interface for manually controlling joints
- RViz2: 3D visualization tool

### 3. Camera Configuration

Camera extrinsics configuration is in `src/robot_description/config/camera_extrinsics.yaml`, containing:
- **camera_position**: Camera position (x, y, z), relative to `base_link`
- **camera_quaternion**: Camera orientation quaternion (qx, qy, qz, qw)

Launch files automatically read parameters from the yaml file and pass them to xacro.

## Robot Description

UR5e is a 6-degree-of-freedom robot arm with the following joints and links:

### Joints
1. **shoulder_pan_joint** - Shoulder pan joint (rotation around Z-axis)
2. **shoulder_lift_joint** - Shoulder lift joint (rotation around Y-axis)
3. **elbow_joint** - Elbow joint (rotation around Y-axis)
4. **wrist_1_joint** - Wrist joint 1 (rotation around Y-axis)
5. **wrist_2_joint** - Wrist joint 2 (rotation around Z-axis)
6. **wrist_3_joint** - Wrist joint 3 (rotation around Y-axis)

### Links
- `base_link`: Base
- `shoulder_link`: Shoulder link
- `upper_arm_link`: Upper arm link (425mm)
- `forearm_link`: Forearm link (392.25mm)
- `wrist_1_link`: Wrist 1 link
- `wrist_2_link`: Wrist 2 link
- `wrist_3_link`: Wrist 3 link
- `ee_link`: End effector connection flange

## Robot Parameters

- **Working radius**: Approximately 850mm
- **Payload**: 5kg
- **Joint range**: ±360° (each joint)

## Camera Configuration

### Hand-to-Eye Calibrated Camera

The workspace includes a hand-to-eye calibrated camera URDF description:
- **camera_link**: Camera main link, fixed to `base_link` (RealSense official)
- **camera_color_optical_frame**: RealSense official optical coordinate frame (Z forward, X right, Y down)

Camera extrinsics are read from `camera_extrinsics.yaml` file:
- Position: Translation from `base_link` → `camera_link`
- Orientation: Using RPY Euler angles (converted from quaternion)

### Camera Parameter Source

Camera extrinsics come from `camera_extrinsics.yaml`:
- Position: `[1.29163, 0.0146956, 0.665489]` meters
- Quaternion: `[-0.396568, -0.0131018, 0.91791, -0.00187618]`
- Equivalent RPY: `[0.0372615, -0.815369, -3.12141]` radians

## Future Development

You can add as needed:
- Real camera mesh files (STL/DAE)
- Camera sensor plugins
- Other sensor descriptions
- End effector descriptions
- MoveIt configuration files
- Physics simulation properties
