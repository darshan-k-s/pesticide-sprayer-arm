# Automation Task Quick Guide

Usage instructions for automated leaf detection and robot arm control system.

## 📋 Quick Start

### Step 1: Start System

Run in one terminal:

```bash
cd /home/hao/Desktop/Hao_MTRN4231
./default_scripts/start_all.sh
```

This will start all required system components:
- Robot driver
- MoveIt planner and RViz
- Collision objects
- Camera TF description
- Camera node
- Leaf detection server

### Step 2: Start Automation Task

After the system is fully started (usually 10-15 seconds), run in another terminal:

```bash
cd /home/hao/Desktop/Hao_MTRN4231
./default_scripts/run_automation.sh
```

The automation task will:
1. Detect leaves in images
2. Obtain coordinates of each leaf
3. Move robot arm to each leaf position

## ⚙️ Custom Parameters

```bash
# Set minimum detection area
./default_scripts/run_automation.sh --min-area 2000

# Combine multiple parameters
./default_scripts/run_automation.sh --min-area 2000 --confidence 0.8

# View help
./default_scripts/run_automation.sh --help
```

## 📊 Workflow

```
Start System → Wait for all nodes ready → Detect leaves → Move robot arm → Process next leaf → Complete
```

## 🔍 Troubleshooting

### Error: "Leaf detection service unavailable"

**Solution:**
- Ensure `./default_scripts/start_all.sh` has been run
- Wait 15 seconds for all nodes to fully start
- Check LeafDetection terminal for error messages

### Error: "Robot arm movement failed"

**Solution:**
- Check MoveitServer terminal for planning errors
- Ensure collision objects are correctly added
- Check RViz for collision warnings
- Try adjusting coordinate bias parameters

### Error: "install/setup.bash file does not exist"

**Solution:**
- First run `./default_scripts/start_all.sh` to build workspace
- Or manually build: `colcon build --symlink-install`

## 📚 More Information

For detailed documentation, refer to:
- [task_automation package documentation](../src/task_automation/README.md)
- [Leaf detection service documentation](../src/detect_leaf_pkg/USAGE_SERVER.md)

## 💡 Tips

- Rebuild after code changes: `colcon build --symlink-install`
- Ensure camera is properly installed and started
- Use `ros2 topic list` and `ros2 service list` to check system status
- Visualize robot movement and leaf detection results in RViz
