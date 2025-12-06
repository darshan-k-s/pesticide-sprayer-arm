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

SKIP_CHECK=false
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
        --skip-check)
            SKIP_CHECK=true
            shift
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
            echo "  --skip-check        Skip system status check and start task directly"
            echo "  --help              Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                              # Use default parameters"
            echo "  $0 --min-area 2000   # Custom parameters"
            echo "  $0 --skip-check      # Skip check and start directly"
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
    echo " Error: install/setup.bash file does not exist!"
    echo "Please run ./default_scripts/start_all.sh first to start the system"
    exit 1
fi
source install/setup.bash || {
    echo " Error: Failed to load workspace!"
    exit 1
}

# Check if required services are running (unless skipped)
if [ "$SKIP_CHECK" = false ]; then
    echo "Checking system status..."
    
    # Try to restart ROS2 daemon if communication fails
    echo "  Verifying ROS2 daemon..."
    if ! ros2 daemon status &>/dev/null; then
        echo "  Starting ROS2 daemon..."
        ros2 daemon start 2>/dev/null || true
        sleep 2
    else
        # Daemon is running, but might need restart if communication fails
        echo "  ROS2 daemon is running"
    fi
    
    # Wait a bit for ROS2 environment to initialize
    sleep 2
    
    # Check service availability with error handling
    check_service_available() {
        local service_name="$1"
        # Suppress stderr to avoid displaying XML-RPC errors during startup
        if timeout 3 ros2 service list 2>/dev/null | grep -q "$service_name" 2>/dev/null; then
            return 0
        else
            return 1
        fi
    }
    
    # Try to check services, with retries and daemon restart
    service_check_attempts=0
    max_service_check_attempts=3
    service_check_success=false
    
    while [ $service_check_attempts -lt $max_service_check_attempts ]; do
        if timeout 3 ros2 service list &>/dev/null; then
            service_check_success=true
            break
        fi
        service_check_attempts=$((service_check_attempts + 1))
        if [ $service_check_attempts -lt $max_service_check_attempts ]; then
            echo "  Communication issue detected, attempting to fix... ($service_check_attempts/$max_service_check_attempts)"
            # Try restarting daemon once
            if [ $service_check_attempts -eq 1 ]; then
                echo "  Restarting ROS2 daemon..."
                ros2 daemon stop 2>/dev/null || true
                sleep 1
                ros2 daemon start 2>/dev/null || true
                sleep 2
            else
                sleep 2
            fi
        fi
    done
    
    if [ "$service_check_success" = false ]; then
        echo ""
        echo "  Warning: Unable to communicate with ROS2 system via daemon"
        echo "This might be a temporary issue. The automation task will still start and wait for services."
        echo ""
        echo "If you continue to have issues, you can:"
        echo "  1. Skip the check: ./run_automation.sh --skip-check"
        echo "  2. Manually restart daemon: ros2 daemon stop && ros2 daemon start"
        echo "  3. Ensure system is running: ./default_scripts/start_all.sh"
        echo ""
        read -p "Continue anyway? (y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
        SKIP_CHECK=true
    else
        # Check for leaf detection service
        if ! check_service_available "leaf_detection_srv"; then
            echo "  Warning: Leaf detection service is not currently visible"
            echo "The automation task will wait for it to become available..."
        else
            echo " Leaf detection service is running normally"
        fi
    fi
else
    echo "Skipping system status check (--skip-check flag used)"
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
    gnome-terminal --title "AutomationTask" -- bash -c "cd \"$WORKSPACE_DIR\" && fix_library_path() { if [ -n \"\$CONDA_PREFIX\" ]; then SYSTEM_LIB_PATH=\"/usr/lib/x86_64-linux-gnu\"; if [ -n \"\$LD_LIBRARY_PATH\" ]; then export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH}\"; else export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib\"; fi; fi; }; fix_library_path && source install/setup.bash && ros2 launch task_automation automation_task.launch.py min_area:=$MIN_AREA confidence:=$CONFIDENCE home_x:=$HOME_X home_y:=$HOME_Y home_z:=$HOME_Z; exec bash"
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
