#!/bin/bash
# Complete system startup script
# Startup sequence: Build -> Source -> Robot Driver -> MoveIt -> Collision Objects -> Arm Monitoring -> Camera TF -> Camera Node -> Dynamic Obstacles Monitor -> Leaf Detection -> Arduino Communication

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
echo "[0/9] Cleaning old build files (if needed)..."
# Clean directories that may cause symbolic link conflicts (only when necessary)
if [ -d "build/arm_msgs/ament_cmake_python/arm_msgs/arm_msgs" ] && [ ! -L "build/arm_msgs/ament_cmake_python/arm_msgs/arm_msgs" ]; then
    echo "  Cleaning arm_msgs symbolic link conflict (directory instead of link)..."
    rm -rf "build/arm_msgs/ament_cmake_python/arm_msgs/arm_msgs" 2>/dev/null || true
fi

echo "[0/9] Building workspace..."
colcon build --symlink-install
BUILD_RESULT=$?

# If build fails, try cleaning and rebuilding once
if [ $BUILD_RESULT -ne 0 ]; then
    echo "Build failed, attempting to clean and rebuild..."
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
echo "[0.5/9] Sourcing workspace..."
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

# 1. Start robot driver + MoveIt with custom URDF (using setupFakeur5e.sh)
echo "[1/9] Starting robot driver + MoveIt with custom URDF..."
gnome-terminal -t "URBringup" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ./default_scripts/setupFakeur5e.sh; exec bash'"

sleep 10

# 2. Add collision objects (needs to be after MoveIt starts)
echo "[2/9] Adding collision objects to scene..."
gnome-terminal -t "CollisionObjects" -e "bash -c 'cd \"$WORKSPACE_DIR\" && fix_library_path() { if [ -n \"\$CONDA_PREFIX\" ]; then SYSTEM_LIB_PATH=\"/usr/lib/x86_64-linux-gnu\"; if [ -n \"\$LD_LIBRARY_PATH\" ]; then export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH}\"; else export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib\"; fi; fi; }; fix_library_path && source install/setup.bash && ros2 launch arm_manipulation add_collision_objects_launch.py; exec bash'"

sleep 2

# 3. Start arm monitoring (needs MoveIt TF and joint states)
echo "[3/9] Starting arm position monitoring..."
gnome-terminal -t "ArmMonitoring" -e "bash -c 'cd \"$WORKSPACE_DIR\" && fix_library_path() { if [ -n \"\$CONDA_PREFIX\" ]; then SYSTEM_LIB_PATH=\"/usr/lib/x86_64-linux-gnu\"; if [ -n \"\$LD_LIBRARY_PATH\" ]; then export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH}\"; else export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib\"; fi; fi; }; fix_library_path && source install/setup.bash && ros2 run arm_monitoring arm_position_viewer; exec bash'"

sleep 2

# 4. Start robot + camera TF description (using verified quaternion)
echo "[4/9] Starting robot + camera TF description..."
gnome-terminal -t "RobotCameraTF" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch robot_description display_with_camera.launch.py; exec bash'"

sleep 3

# 5. Start camera node
echo "[5/9] Starting RealSense camera node..."
gnome-terminal -t "Camera" -e "bash -c 'cd \"$SCRIPT_DIR\" && ./camera.sh; exec bash'"

sleep 3  # Give camera time to initialize

# 6. Start dynamic obstacles monitor (needs MoveIt to be running)
echo "[6/9] Starting dynamic obstacles monitor..."
gnome-terminal -t "ObstacleMonitor" -e "bash -c 'cd \"$WORKSPACE_DIR\" && fix_library_path() { if [ -n \"\$CONDA_PREFIX\" ]; then SYSTEM_LIB_PATH=\"/usr/lib/x86_64-linux-gnu\"; if [ -n \"\$LD_LIBRARY_PATH\" ]; then export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH}\"; else export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib\"; fi; fi; }; fix_library_path && source install/setup.bash && ros2 run arm_manipulation dynamic_obstacle_control; exec bash'"

sleep 2

# 7. Start leaf detection server (wait for dependencies to be ready to avoid coordinate errors)
echo "[7/9] Waiting for dependencies to be ready..."
echo "  Checking TF availability (camera_color_optical_frame -> base_link)..."
MAX_WAIT=30  # Maximum wait time in seconds
TF_READY=false
CAMERA_READY=false

for i in $(seq 1 $MAX_WAIT); do
    # Check TF transform - try to get transform, any output means TF is working
    if [ "$TF_READY" = false ]; then
        TF_OUTPUT=$(timeout 3 ros2 run tf2_ros tf2_echo camera_color_optical_frame base_link --once 2>&1)
        if echo "$TF_OUTPUT" | grep -qE "(At time|Translation|transform)" 2>/dev/null; then
            TF_READY=true
            echo "  ✓ TF transforms ready after ${i} seconds"
        fi
    fi
    
    # Check camera topics - check if topics exist and have publishers
    if [ "$CAMERA_READY" = false ]; then
        # Check if topics exist
        if ros2 topic list 2>/dev/null | grep -qE "/camera/camera/color/(image_raw|camera_info)" 2>/dev/null; then
            # Check if camera_info has a publisher (means camera is actually publishing)
            CAMERA_INFO=$(timeout 2 ros2 topic info /camera/camera/color/camera_info 2>/dev/null)
            if echo "$CAMERA_INFO" | grep -qE "Publisher count: [1-9]" 2>/dev/null; then
                CAMERA_READY=true
                echo "  ✓ Camera topics ready after ${i} seconds"
            fi
        fi
    fi
    
    # If both are ready, break
    if [ "$TF_READY" = true ] && [ "$CAMERA_READY" = true ]; then
        echo "  ✓ All dependencies ready!"
        break
    fi
    
    if [ $((i % 5)) -eq 0 ]; then
        echo "  ⏳ Still waiting... TF: $([ "$TF_READY" = true ] && echo "✓" || echo "✗"), Camera: $([ "$CAMERA_READY" = true ] && echo "✓" || echo "✗") (${i}/${MAX_WAIT}s)"
    fi
    sleep 1
done

if [ "$TF_READY" = false ] || [ "$CAMERA_READY" = false ]; then
    echo "  Warning: Some dependencies may not be fully ready"
    if [ "$TF_READY" = false ]; then
        echo "     - TF transforms not ready"
        echo "       Trying to diagnose:"
        echo "         Checking if TF frames exist..."
        ros2 run tf2_ros tf2_echo camera_color_optical_frame base_link --once 2>&1 | head -5 || echo "         ✗ Cannot get TF transform"
    fi
    if [ "$CAMERA_READY" = false ]; then
        echo "     - Camera topics not ready"
        echo "       Trying to diagnose:"
        echo "         Checking camera topics..."
        if ros2 topic list 2>/dev/null | grep -q "/camera"; then
            echo "         ✓ Camera topics exist, checking publishers..."
            ros2 topic info /camera/camera/color/camera_info 2>/dev/null | grep "Publisher" || echo "         ✗ No publishers found"
        else
            echo "         ✗ Camera topics not found"
        fi
    fi
    echo "  If you see coordinate errors, restart this script (./default_scripts/start_all.sh)"
    echo "  Continuing anyway - dependencies may become available shortly..."
    sleep 3  # Give a bit more time even if check failed
else
    echo "  ✓ All dependencies confirmed ready"
    sleep 2
fi

echo "[7/9] Starting leaf detection server..."
gnome-terminal -t "LeafDetection" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 launch detect_leaf_pkg leaf_detection_server.launch.py; exec bash'"

sleep 2

# 8. Start Arduino communication service
echo "[8/9] Starting Arduino communication service..."
gnome-terminal -t "ArduinoServer" -e "bash -c 'cd \"$WORKSPACE_DIR\" && source install/setup.bash && ros2 run arduino_communication leafServerNode; exec bash'"

sleep 2

echo ""
echo "=========================================="
echo "All nodes started!"
echo "- URBringup: Robot Driver + MoveIt + RViz (with custom URDF)"
echo "- CollisionObjects: Collision Objects"
echo "- ArmMonitoring: Arm Position Viewer"
echo "- RobotCameraTF: Robot + Camera TF"
echo "- Camera: RealSense Camera Node"
echo "- ObstacleMonitor: Dynamic Obstacles Monitor (subscribes to /obsFromImg)"
echo "- LeafDetection: Leaf Detection Server + Visualization"
echo "- ArduinoServer: Arduino Communication Service (vacuum/spray control)"
echo "=========================================="
echo ""
echo "Tip: Use ./default_scripts/run_automation.sh to start automation task"

# ros2 launch arm_manipulation moveit_scene_home_launch.py x:=0.25 y:=0.10 z:=0.55
