#!/bin/bash
# Automation task runner script
# Run automated leaf detection and robot arm movement

echo "=========================================="
echo "Starting Automation Task"
echo "=========================================="

# Get script directory, then get workspace root directory
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

# Parse command line arguments first (before checking system)
MIN_AREA="0.0"
CONFIDENCE="0.0"
HOME_X="0.25"
HOME_Y="0.10"
HOME_Z="0.55"

while [[ $# -gt 0 ]]; do
    case $1 in
        --min-area)
            MIN_AREA="$2"
            shift 2
            ;;
        --confidence)
            CONFIDENCE="$2"
            shift 2
            ;;
        --home-x)
            HOME_X="$2"
            shift 2
            ;;
        --home-y)
            HOME_Y="$2"
            shift 2
            ;;
        --home-z)
            HOME_Z="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --min-area VALUE     Minimum leaf area threshold for detection (default: 0.0)"
            echo "  --confidence VALUE  Leaf detection confidence threshold (default: 0.0)"
            echo "  --home-x VALUE      Home position X coordinate in meters (default: 0.25)"
            echo "  --home-y VALUE      Home position Y coordinate in meters (default: 0.10)"
            echo "  --home-z VALUE      Home position Z coordinate in meters (default: 0.55)"
            echo "  --help              Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                              # Use default parameters"
            echo "  $0 --min-area 2000   # Custom parameters"
            echo "  $0 --home-x 0.3 --home-y 0.2 --home-z 0.6   # Custom home position"
            exit 0
            ;;
        *)
            echo "Unknown parameter: $1"
            echo "Use --help to view help information"
            exit 1
            ;;
    esac
done

# Source workspace
echo "Loading workspace..."
if [ ! -f "install/setup.bash" ]; then
    echo "❌ Error: install/setup.bash file does not exist!"
    echo "Please run ./default_scripts/start_all.sh first to start the system"
    exit 1
fi
source install/setup.bash || {
    echo "❌ Error: Failed to load workspace!"
    exit 1
}

# Check if required services are running
echo "Checking system status..."
sleep 2

if ! ros2 service list | grep -q "leaf_detection_srv"; then
    echo "⚠️  Warning: Leaf detection service is not running"
    echo "Waiting for service to start... (up to 30 seconds)"
    
    # Wait up to 30 seconds for service
    for i in {1..30}; do
        if ros2 service list | grep -q "leaf_detection_srv"; then
            echo "✓ Leaf detection service is ready"
            break
        fi
        sleep 1
        if [ $i -eq 30 ]; then
            echo "❌ Error: Leaf detection service timeout"
            echo "Please ensure ./default_scripts/start_all.sh has been run to start all system components"
            exit 1
        fi
    done
else
    echo "✓ Leaf detection service is running normally"
fi

# Start automation task
echo ""
echo "=========================================="
echo "Starting Automation Task"
echo "=========================================="
echo "Parameters:"
echo "  - Minimum area: $MIN_AREA"
echo "  - Confidence: $CONFIDENCE"
echo "  - Home position: x=$HOME_X, y=$HOME_Y, z=$HOME_Z m"
echo "=========================================="
echo ""

# Run in new terminal or current terminal
if command -v gnome-terminal &> /dev/null; then
    echo "Starting automation task in new terminal..."
    gnome-terminal -t "AutomationTask" -e "bash -c 'cd \"$WORKSPACE_DIR\" && fix_library_path() { if [ -n \"\$CONDA_PREFIX\" ]; then SYSTEM_LIB_PATH=\"/usr/lib/x86_64-linux-gnu\"; if [ -n \"\$LD_LIBRARY_PATH\" ]; then export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH}\"; else export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib\"; fi; fi; }; fix_library_path && source install/setup.bash && ros2 launch task_automation automation_task.launch.py min_area:=$MIN_AREA confidence:=$CONFIDENCE home_x:=$HOME_X home_y:=$HOME_Y home_z:=$HOME_Z; exec bash'"
    echo "Automation task started in new terminal"
else
    echo "Starting automation task in current terminal..."
    ros2 launch task_automation automation_task.launch.py \
        min_area:=$MIN_AREA \
        confidence:=$CONFIDENCE \
        home_x:=$HOME_X \
        home_y:=$HOME_Y \
        home_z:=$HOME_Z
fi
