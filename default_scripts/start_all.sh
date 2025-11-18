#!/bin/bash
# Complete system startup script
# Startup sequence: Build -> Source -> Robot Driver -> MoveIt -> Collision Objects -> Arm Monitoring -> Camera TF -> Camera Node -> Leaf Detection

echo "=========================================="
echo "Starting UR5e + RealSense Complete System"
echo "=========================================="

# Get script directory, then get workspace root directory (parent directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WORKSPACE_DIR"

# Fix Conda and ROS2 library conflicts: prioritize system libraries
fix_library_path() {
    if [ -n "$CONDA_PREFIX" ]; then
        SYSTEM_LIB_PATH="/usr/lib/x86_64-linux-gnu"
        if [ -n "$LD_LIBRARY_PATH" ]; then
            export LD_LIBRARY_PATH="${SYSTEM_LIB_PATH}:${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH}"
        else
            export LD_LIBRARY_PATH="${SYSTEM_LIB_PATH}:${CONDA_PREFIX}/lib"
        fi
    fi
}

# Fix library path at script startup
fix_library_path

# 0. Clean and build workspace
echo "[0/8] Cleaning old build files (if needed)..."
# Clean directories that may cause symbolic link conflicts (only when necessary)
if [ -d "build/arm_msgs/ament_cmake_python/arm_msgs/arm_msgs" ] && [ ! -L "build/arm_msgs/ament_cmake_python/arm_msgs/arm_msgs" ]; then
    echo "  Cleaning arm_msgs symbolic link conflict (directory instead of link)..."
    rm -rf "build/arm_msgs/ament_cmake_python/arm_msgs/arm_msgs" 2>/dev/null || true
fi

echo "[0/8] Building workspace..."
colcon build --symlink-install
BUILD_RESULT=$?

# If build fails, try cleaning and rebuilding once
if [ $BUILD_RESULT -ne 0 ]; then
    echo "⚠️  Build failed, attempting to clean and rebuild..."
    echo "  Cleaning build and install directories..."
    rm -rf build/ install/ log/ 2>/dev/null || true
    echo "  Rebuilding..."
    colcon build --symlink-install
    BUILD_RESULT=$?
fi

if [ $BUILD_RESULT -ne 0 ]; then
    echo "❌ Error: Build failed!"
    echo "Please check build error messages and fix before retrying."
    exit 1
fi

# Source install
echo "[0.5/8] Sourcing workspace..."
if [ ! -f "install/setup.bash" ]; then
    echo "❌ Error: install/setup.bash file does not exist!"
    echo "Please ensure build succeeded before running this script."
    exit 1
fi
source install/setup.bash || {
    echo "❌ Error: Failed to source workspace!"
    exit 1
}

sleep 2

# 1. Start robot driver and MoveIt (setupFakeur5e.sh will handle)
# echo "[1/8] Starting robot driver..."
# gnome-terminal -t "DriverServer" -e 'bash -c "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=scaled_joint_trajectory_controller use_fake_hardware:=true launch_rviz:=false; exec bash"'

# sleep 5

# echo "[2/8] Starting MoveIt + RViz..."
# gnome-terminal -t "MoveitServer" -e 'bash -c "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_fake_hardware:=true; exec bash"'

echo "[1/8] Starting robot driver + MoveIt with custom URDF..."
gnome-terminal -t "URBringup" -e "bash -c './default_scripts/setupFakeur5e.sh; exec bash'"

sleep 5

# 2. Add collision objects (needs to be after MoveIt starts)
echo "[3/8] Adding collision objects to scene..."
gnome-terminal -t "CollisionObjects" -e "bash -c 'cd \"$WORKSPACE_DIR\" && fix_library_path() { if [ -n \"\$CONDA_PREFIX\" ]; then SYSTEM_LIB_PATH=\"/usr/lib/x86_64-linux-gnu\"; if [ -n \"\$LD_LIBRARY_PATH\" ]; then export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH}\"; else export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib\"; fi; fi; }; fix_library_path && source install/setup.bash && ros2 launch arm_manipulation add_collision_objects_launch.py; exec bash'"

sleep 2

# 3. Start arm monitoring (needs MoveIt TF and joint states)
echo "[4/8] Starting arm position monitoring..."
gnome-terminal -t "ArmMonitoring" -e "bash -c 'cd \"$WORKSPACE_DIR\" && fix_library_path() { if [ -n \"\$CONDA_PREFIX\" ]; then SYSTEM_LIB_PATH=\"/usr/lib/x86_64-linux-gnu\"; if [ -n \"\$LD_LIBRARY_PATH\" ]; then export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH}\"; else export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib\"; fi; fi; }; fix_library_path && source install/setup.bash && ros2 run arm_monitoring arm_position_viewer; exec bash'"

sleep 2

# 4. Start robot + camera TF description
echo "[5/8] Starting robot + camera TF description..."
gnome-terminal -t "RobotCameraTF" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch robot_description display_with_camera.launch.py; exec bash'"

sleep 3

# 5. Start camera node
echo "[6/8] Starting RealSense camera node..."
gnome-terminal -t "Camera" -e "bash -c 'cd \"$SCRIPT_DIR\" && ./camera.sh; exec bash'"

sleep 2

# 6. Start leaf detection server
echo "[7/8] Starting leaf detection server..."
gnome-terminal -t "LeafDetection" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch detect_leaf_pkg leaf_detection_server.launch.py; exec bash'"

sleep 2

echo ""
echo "=========================================="
echo "All nodes started!"
echo "- DriverServer: Robot Driver"
echo "- MoveitServer: MoveIt + RViz"
echo "- CollisionObjects: Collision Objects"
echo "- ArmMonitoring: Arm Position Viewer"
echo "- RobotCameraTF: Robot + Camera TF"
echo "- Camera: RealSense Camera Node"
echo "- LeafDetection: Leaf Detection Server + Visualization"
echo "=========================================="
echo ""
echo "💡 Tip: Use ./default_scripts/run_automation.sh to start automation task"

# ros2 launch arm_manipulation moveit_scene_home_launch.py x:=0.25 y:=0.10 z:=0.55