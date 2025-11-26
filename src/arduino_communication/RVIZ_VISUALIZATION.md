# RViz2 Pump Status Visualization Guide

## Feature Description

`leafServerNode` now automatically publishes pump on/off status to RViz2, displayed as text in the 3D view.

## Display Content

- **Vacuum Pump**: Displayed at z=1.5m position
  - Green text = ON
  - Red text = OFF

- **Spray Pump**: Displayed at z=1.35m position
  - Green text = ON
  - Red text = OFF

## Enable Visualization in RViz2

1. **Start the system** (ensure `leafServerNode` is running)

2. **Open RViz2** (usually auto-started via MoveIt launch file)

3. **Add Marker display**:
   - Click the "Add" button in the left panel
   - Select "By topic"
   - Find and select `/pump_status_marker` topic
   - Click "OK"

4. **View status**:
   - Text will be displayed in the 3D view above the robot
   - Status updates in real-time (every 100ms)
   - Text color updates immediately when pump status changes

## Text Position

- Text is displayed in the `base_link` coordinate frame
- Position: x=0.0, y=0.0, z=1.5m (vacuum pump) and z=1.35m (spray pump)
- Text always faces the camera (TEXT_VIEW_FACING type)

## Custom Position

To adjust text display position, modify the `update_visualization()` method in `leafServer.cpp`:

```cpp
vacuum_marker.pose.position.x = 0.0;  // X coordinate
vacuum_marker.pose.position.y = 0.0;  // Y coordinate
vacuum_marker.pose.position.z = 1.5;  // Z coordinate (height)
```

## Notes

- Ensure RViz2's Fixed Frame is set to `base_link` or the appropriate coordinate frame
- If text is not visible, check if RViz2's Marker display is enabled
- Text size can be adjusted via the `scale.z` parameter (currently 0.15)
