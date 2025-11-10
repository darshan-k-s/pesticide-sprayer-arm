# Task Automation Package

Automated task management package integrating leaf detection and robot arm control for complete automated workflow.

## 📋 Feature Overview

This package provides an automation orchestrator that can:
1. **Call leaf detection service** - Detect leaves in images and obtain coordinates
2. **Move robot arm** - Automatically move robot arm to each position based on detected leaf coordinates
3. **Sequential processing** - Automatically process multiple detected leaves one by one

## 🏗️ Package Structure

```
task_automation/
├── task_automation/
│   ├── __init__.py
│   └── automation_orchestrator.py  # Main control logic
├── launch/
│   └── automation_task.launch.py   # Launch file
├── resource/
│   └── task_automation
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 🚀 Usage

### Method 1: Use automation task script (Recommended)

**Two-step workflow:**

1. **First, start all system components:**
```bash
cd /home/hao/Desktop/Hao_MTRN4231
./default_scripts/start_all.sh
```

2. **Then start automation task in another terminal:**
```bash
cd /home/hao/Desktop/Hao_MTRN4231
./default_scripts/run_automation.sh
```

The script will automatically check system status, wait for services to be ready, then start the automation task.

### Method 2: Use custom parameters

The automation task script supports custom parameters:

```bash
./default_scripts/run_automation.sh --min-area 2000 --confidence 0.0
```

View all available parameters:
```bash
./default_scripts/run_automation.sh --help
```

#### Parameter Description:

- `--min-area VALUE`: Minimum leaf area threshold for detection (pixels²), default `0.0`
- `--confidence VALUE`: Leaf detection confidence threshold, default `0.0`
- `--home-x VALUE`: Home position X coordinate (meters), default `0.25`
- `--home-y VALUE`: Home position Y coordinate (meters), default `0.10`
- `--home-z VALUE`: Home position Z coordinate (meters), default `0.55`

### Method 3: Manually launch automation task

If the system is already running, you can launch directly using ROS2 command:

```bash
cd /home/hao/Desktop/Hao_MTRN4231
source install/setup.bash
ros2 launch task_automation automation_task.launch.py
```

With parameters:
```bash
ros2 launch task_automation automation_task.launch.py \
    min_area:=2000.0 \
    confidence:=0.0 \
    home_x:=0.25 \
    home_y:=0.10 \
    home_z:=0.55
```

### Method 4: Run executable directly

```bash
source install/setup.bash
ros2 run task_automation automation_orchestrator --ros-args \
    -p min_area:=2000.0 \
    -p confidence:=0.0 \
    -p home_x:=0.25 \
    -p home_y:=0.10 \
    -p home_z:=0.55
```

## 🔄 Workflow

The automation orchestrator executes the following steps:

1. **Wait for service ready** - Wait for leaf detection service to be available (timeout 30 seconds)
2. **Detect leaves** - Call leaf detection service to obtain coordinates of all leaves
3. **Log results** - Print number of detected leaves and position information
4. **Process each leaf** - For each detected leaf:
   - Move robot arm to leaf position (apply coordinate bias)
   - After picking leaf, move to trash bin to discard
   - Wait 2 seconds
   - Process next leaf
5. **Return to home** - After all leaves are processed, robot arm automatically returns to home position
6. **Task summary** - Display number of successfully processed leaves

## 🗑️ Trash Bin Visualization

Trash bin is added to MoveIt scene as a collision object:
- **Position**: x=0.10m, y=0.50m, z=0.25m (center height)
- **Size**: 0.3m × 0.3m × 0.4m
- **Visible in RViz**: After starting the system, trash bin will be displayed as orange collision box in the scene

## 🔍 Dependencies

This package depends on the following ROS2 packages and services:

**ROS2 Packages:**
- `rclpy` - ROS2 Python client library
- `geometry_msgs` - Geometry message types
- `arm_msgs` - Custom message and service definitions

**Runtime Dependencies:**
- `detect_leaf_pkg` - Provides `leaf_detection_srv` service
- `arm_manipulation` - Provides `move_arm_to_pose` node
- MoveIt configuration and robot driver

## 📝 Example Output

```
================================================
🌿 Leaf Detection Results
================================================
Status: Detection successful
Leaves detected: 3
  Leaf 1: X=0.250m, Y=0.100m, Z=0.550m
  Leaf 2: X=0.300m, Y=-0.150m, Z=0.500m
  Leaf 3: X=0.400m, Y=0.200m, Z=0.520m
================================================

Processing leaf 1/3...
Moving robot arm to position: x=0.250m, y=0.100m, z=0.600m
✓ Robot arm movement successful
✓ Leaf 1 processed successfully
Waiting 2s before processing next leaf...

Processing leaf 2/3...
...

================================================
Automation task complete
Successfully processed: 3/3 leaves
================================================
```

## ⚠️ Notes

1. **System must be started first** - Before using automation tasks, ensure all required system components are running:
   - Robot driver
   - MoveIt planner and RViz
   - Collision objects
   - Camera TF description
   - Camera node
   - Leaf detection server

2. **Coordinate bias** - Coordinate bias parameters (bias_x, bias_y, bias_z) can be adjusted according to actual calibration needs.

3. **Wait time** - Wait 2 seconds between processing leaves. To modify, edit `wait_between_leaves` parameter in launch file or `automation_orchestrator.py`.

4. **Error handling** - If processing of a leaf fails, the task will continue with the next leaf.

## 🔧 Development and Debugging

### Rebuild package

```bash
cd /home/hao/Desktop/Hao_MTRN4231
colcon build --packages-select task_automation --symlink-install
source install/setup.bash
```

### View available nodes

```bash
ros2 pkg executables task_automation
```

### View node parameters

```bash
ros2 param describe /automation_orchestrator
```

### Monitor ROS2 topics and services

```bash
# List all topics
ros2 topic list

# List all services
ros2 service list

# View leaf detection service info
ros2 service info /leaf_detection_srv
```

## 📚 Related Documentation

- [detect_leaf_pkg](../detect_leaf_pkg/USAGE_SERVER.md) - Leaf detection service documentation
- [arm_manipulation](../arm_manipulation/) - Robot arm control package
- [start_all.sh](../../default_scripts/start_all.sh) - Complete system startup script

## 📝 License

MIT License

## 👥 Maintainer

hao
