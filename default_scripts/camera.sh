#!/bin/bash
# RealSense Camera Launch Script

# First, terminate any existing old processes
echo "Checking and terminating old RealSense processes..."
pkill -f realsense2_camera 2>/dev/null
sleep 1

# Check if device is available
if ! lsusb | grep -qi "intel.*realsense"; then
    echo "Error: RealSense camera device not detected"
    exit 1
fi

# Start RealSense camera node
echo "Starting RealSense camera..."
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    align_depth.enable:=true \
    pointcloud.enable:=true \
    color_width:=640 \
    color_height:=480 \
    depth_width:=640 \
    depth_height:=480
