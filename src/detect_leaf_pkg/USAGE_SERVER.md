# Leaf Detection Service Usage Guide

## 🚀 Quick Start

### 1. Build Workspace

```bash
cd /home/hao/Desktop/Test/pesticide_robot_ws

# First build the service interface package
colcon build --packages-select arm_msgs
source install/setup.bash

# Then build the detection package
colcon build --packages-select detect_leaf_pkg
source install/setup.bash
```

### 2. Start Server

**Method A - Using Launch File (Recommended):**
```bash
ros2 launch detect_leaf_pkg leaf_detection_server.launch.py
```

This will start both:
- `leaf_detection_server` - Detection server
- `leaf_visualization_node` - Visualization node

**Method B - Disable Visualization Node:**
```bash
ros2 launch detect_leaf_pkg leaf_detection_server.launch.py enable_visualization:=false
```

**Method C - Run Server Directly:**
```bash
ros2 run detect_leaf_pkg leaf_detection_server
```

---

## 📡 Call Detection Service

### Method 1: Using Client Program (Interactive)

```bash
ros2 run detect_leaf_pkg leaf_detection_client
```

Then follow the prompts to enter parameters:
```
Enter command (min_area confidence) or 'exit' to quit: 2000 0.0
```

**Parameter Description:**
- `min_area`: Minimum leaf area threshold (pixels²)
  - `0` = Use default value 2000
  - `2000` = Set minimum area to 2000 pixels²
- `confidence`: Confidence threshold (currently unused, keep as 0.0)

**Example Input:**
```
2000 0.0    # Use min_area=2000
0.0         # Use default parameters (only enter one number as min_area)
2000        # Same as above
```

### Method 2: Command Line Direct Call

```bash
ros2 service call /leaf_detection_srv arm_msgs/srv/LeafDetectionSrv \
  "{command: 'detect', min_area: 2000.0, confidence: 0.0}"
```

### Method 3: Python Script Call

Use the existing example script:
```bash
cd /home/hao/Desktop/Test/pesticide_robot_ws
source install/setup.bash
python3 test_leaf_detection.py 2000
```

Or create a custom script:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arm_msgs.srv import LeafDetectionSrv

class LeafDetectionClient(Node):
    def __init__(self):
        super().__init__('client')
        self.client = self.create_client(LeafDetectionSrv, 'leaf_detection_srv')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
    
    def detect(self, min_area=2000.0):
        request = LeafDetectionSrv.Request()
        request.command = "detect"
        request.min_area = min_area
        request.confidence = 0.0
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print(f"Detected {response.num_leaves} leaves")
                for i, point in enumerate(response.coordinates):
                    print(f"Leaf {i+1}: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f})")
                return response
        return None

def main():
    rclpy.init()
    client = LeafDetectionClient()
    client.detect(min_area=2000.0)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 📊 Service Response Description

Service return data structure:

```python
response.success       # bool: Whether detection succeeded
response.message       # string: Status message
response.num_leaves    # int32: Number of detected leaves
response.coordinates   # geometry_msgs/Point[]: Leaf coordinates array (base frame)
response.debug_info   # string: Debug information (JSON format)
```

**Important:**
- Coordinates in `response.coordinates` are already in **base frame** coordinates
- Each `Point` contains `x`, `y`, `z` fields (unit: meters)

---

## 🎯 Complete Usage Example

### Terminal 1: Start Camera (if needed)
```bash
ros2 launch realsense2_camera rs_launch.py
```

### Terminal 2: Start Detection Server
```bash
cd /home/hao/Desktop/Test/pesticide_robot_ws
source install/setup.bash
ros2 launch detect_leaf_pkg leaf_detection_server.launch.py
```

You will see:
```
🌿 Leaf Detection Server ready
📡 Service /leaf_detection_srv available
🎨 Leaf Visualization Node started
```

### Terminal 3: Call Detection Service
```bash
source install/setup.bash

# Use client
ros2 run detect_leaf_pkg leaf_detection_client

# Or call directly
ros2 service call /leaf_detection_srv arm_msgs/srv/LeafDetectionSrv \
  "{command: 'detect', min_area: 2000.0, confidence: 0.0}"
```

---

## 📈 Example Output

After successful call, you will see output similar to:

```
🌿 Leaf Detection Results
================================================================================
Status: Detected 2 leaves
Leaves found: 2
  Leaf 1: X=  0.523m, Y= -0.234m, Z=  0.456m
  Leaf 2: X=  0.612m, Y= -0.189m, Z=  0.478m
================================================================================
```

---

## 🔍 Check Service Status

### Check if Service is Available
```bash
ros2 service list | grep leaf_detection
# Should see: /leaf_detection_srv
```

### Check Service Type
```bash
ros2 service type /leaf_detection_srv
# Should see: arm_msgs/srv/LeafDetectionSrv
```

### Check Service Definition
```bash
ros2 interface show arm_msgs/srv/LeafDetectionSrv
```

### Check Node Status
```bash
ros2 node list
# Should see: leaf_detection_server
```

---

## 🎨 Visualization

The visualization node starts automatically (unless disabled), publishing the following topics:

1. **Annotated Image**: `/leaf_detection/annotated_image`
   - Subscribe in RViz to view annotated detection images

2. **RViz Markers**: `/leaf_detection/leaf_markers`
   - Subscribe in RViz to view 3D leaf position markers

### View in RViz

1. Start RViz:
```bash
rviz2
```

2. Add Displays:
   - **Image Display**: Subscribe to `/leaf_detection/annotated_image`
   - **MarkerArray Display**: Subscribe to `/leaf_detection/leaf_markers`

---

## ⚠️ Troubleshooting

### Issue 1: Service Not Available

**Error**: `Service not available, waiting again...`

**Solution:**
- Ensure server is started
- Check nodes: `ros2 node list`
- Check services: `ros2 service list`
- Check logs for errors

### Issue 2: No Leaves Detected

**Possible Causes:**
- Camera not started or no image
- No green objects in image
- `min_area` parameter set too large

**Solution:**
- Check camera: `ros2 topic echo /camera/camera/color/image_raw --once`
- Lower `min_area`: try `500` or `0`
- View debug information: `response.debug_info`

### Issue 3: Build Error

**Error**: `No module named 'arm_msgs'`

**Solution:**
```bash
# Ensure arm_msgs is built first
colcon build --packages-select arm_msgs
source install/setup.bash
colcon build --packages-select detect_leaf_pkg
```

---

## 💡 Usage Tips

1. **Adjust Detection Parameters**:
   - If too many false positives → increase `min_area`
   - If nothing detected → decrease `min_area` or set to 0

2. **Batch Detection**:
   - Write loop scripts to call service multiple times

3. **Save Results**:
   - Save `response.coordinates` to file in client script

---

## 📝 Differences from Old Version

| Feature | Old Version | New Version (Service Architecture) |
|---------|-------------|-----------------------------------|
| Data Acquisition | Subscribe to topics for continuous reception | On-demand service calls |
| Number of Topics | 9 topics | 2 visualization topics |
| Startup Method | `leaf_detector` | `leaf_detection_server` |
| Call Method | Automatic reception | Service calls |
| Resource Usage | Continuous processing | On-demand processing |

---

## 🎯 Next Steps

- Use detection results for robot arm control
- Integrate into your application
- Adjust detection parameters as needed
