# Standalone Leaf Detection Script Usage Guide

## Overview

`standalone_leaf_detection.py` is an independent Python script for detecting leaves and displaying results, **without ROS2 dependency**. It reads images and depth information directly from the RealSense camera, uses PlantCV for detection, and displays results via OpenCV.

## Features

- ✅ No ROS2 dependency
- ✅ Direct RealSense camera data reading
- ✅ PlantCV-based leaf detection
- ✅ Real-time result display
- ✅ 3D coordinate calculation (camera coordinate system)
- ✅ Customizable minimum area threshold

## Requirements

### Python Packages
```bash
pip install opencv-python numpy plantcv pyrealsense2
```

### Hardware Requirements
- Intel RealSense camera (D435/D435i, etc.)
- USB 3.0 connection

## Usage

### Basic Usage
```bash
python standalone_leaf_detection.py
```

### Custom Minimum Area Threshold
```bash
python standalone_leaf_detection.py --min-area 3000
```

### Parameter Description
- `--min-area`: Minimum leaf area threshold (default: 2000)
  - Smaller values detect more leaves (including smaller ones)
  - Larger values only detect larger leaves

## Operation Instructions

1. **Start script**: Run the above command
2. **View detection results**: A window will open showing real-time detection results
3. **Exit program**: Press `q` key to exit

## Detection Result Description

- **Bounding box**: Colored rectangles marking detected leaves
- **Label**: Each leaf has a number (Leaf 1, Leaf 2, ...)
- **Center point**: Leaf center position marked with circles
- **3D coordinates**: Shows Z coordinate (depth, unit: meters)
- **Total count**: Top-left corner shows total detected leaves

## Differences from ROS2 Version

| Feature | ROS2 Version | Standalone Version |
|---------|-------------|-------------------|
| ROS2 dependency | ✅ Required |  Not required |
| Camera input | ROS2 topics | Direct RealSense API |
| Coordinate transform | TF system (base_link) | Camera coordinate system only |
| Visualization | RViz + image topics | OpenCV window |
| Service interface | ROS2 service | None |

## Notes

1. **Coordinate system**: 
   - Standalone version only provides 3D coordinates in camera coordinate system (camera_color_optical_frame)
   - For base_link coordinates, use the ROS2 version

2. **Performance**:
   - Standalone version usually has better performance (no ROS2 overhead)
   - Suitable for quick testing and demos

3. **Camera configuration**:
   - Script automatically configures camera to 640x480 resolution
   - Depth and color streams are automatically aligned

## Troubleshooting

### Camera Not Detected
```
 Camera initialization failed: ...
```
**Solution**: 
- Check camera USB connection
- Ensure camera driver is installed
- Check if another program is using the camera

### No Leaves Detected
- Adjust `--min-area` parameter
- Check lighting conditions
- Ensure leaves are green (HSV range: [40, 60, 40] - [80, 255, 255])

### Invalid Depth Values
- Ensure camera is within 10cm-2m range from target
- Check if depth stream is working properly

## Example Output

```
 Standalone leaf detector initialized
 RealSense camera initialized
  Resolution: 640x480
  Intrinsics: fx=616.23, fy=616.23

Starting detection loop...
Press 'q' to exit

Frame 30: Detected 3 leaves
  Leaf 1: X=0.123m, Y=-0.045m, Z=0.567m
  Leaf 2: X=0.234m, Y=-0.012m, Z=0.589m
  Leaf 3: X=0.345m, Y=0.023m, Z=0.601m
```

## Code Structure

- `StandaloneLeafDetector`: Main detection class
  - `setup_camera()`: Initialize RealSense camera
  - `get_frames()`: Get synchronized color and depth frames
  - `detect_leaves()`: Detect leaves using PlantCV
  - `draw_annotations()`: Draw annotations on image
  - `pixel_to_3d()`: Convert pixel coordinates to 3D coordinates
  - `run()`: Main loop

## License

Same as main project
