# Leaf Sorting and Sprayer Arm

This repository contains the ROS 2 Humble workspace for a UR5e-based leaf sorting and sprayer arm.
The system uses a pole-mounted Intel RealSense camera and a custom sprayer/vacuum end-effector to detect leaves in 3D, plan collision-free trajectories with MoveIt, and pick-and-place diseased leaves into a box or actuate a pump via an Arduino bridge to spray pesticide on healthy leaves – all in a closed-loop pipeline.

The project was developed and tested on **Ubuntu 22.04 + ROS 2 Humble** with a **UR5e**, **RealSense RGB-D camera**, and a **custom-built pesticide sprayer/vacuum end-effector**.

> **Demo:**

![Short Obstacle Run](./media/Obstacle%20Small%20Run%20GIF.gif)



---

## Table of Contents

- [Leaf Sorting and Sprayer Arm](#leaf-sorting-and-sprayer-arm)
  - [Table of Contents](#table-of-contents)
- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
- [Technical Components](#technical-components)
  - [Computer Vision](#computer-vision)
  - [Custom End-Effector](#custom-end-effector)
  - [System Visualisation](#system-visualisation)
  - [Closed-Loop Operation](#closed-loop-operation)
- [Installation and Setup](#installation-and-setup)
- [Running the System](#running-the-system)
- [Results and Demonstration](#results-and-demonstration)
- [Discussion and Future Work](#discussion-and-future-work)
- [Contributors and Roles](#contributors-and-roles)
- [Repository Structure](#repository-structure)
- [References and Acknowledgements](#references-and-acknowledgements)



# Project Overview

Inspecting plant health and selectively treating leaves in greenhouses or controlled environments is still mostly done by hand. Operators must visually identify diseased leaves, remove them, and apply pesticide to healthy foliage – a process that is slow, repetitive, and difficult to perform consistently over time. This project targets **greenhouse operators, horticulture researchers, and high-value crop producers** who need an automated system that can both **treat healthy leaves** and **remove unhealthy ones** in a precise, repeatable way.

The system uses a **UR5e** manipulator with a pole-mounted **Intel RealSense RGB-D camera** overlooking the workspace and a **custom dual-function end-effector**:

- The **pole-mounted camera** captures colour and depth images of the plant area from a fixed overhead viewpoint.
- A perception node detects leaves in the RGB image, classifies them as healthy or unhealthy (based on configured criteria), and estimates their 3D positions.
- Leaf coordinates are transformed from the camera frame into the robot `base_link` frame using calibrated camera–robot extrinsics.
- A task automation node selects targets and calls MoveIt to plan collision-free trajectories for the UR5e.

At the end-effector:
- For **healthy leaves**, the sprayer head is positioned above the target and **pesticide is applied**.
- For **unhealthy or damaged leaves**, the **vacuum gripper** grips the leaf, removes it, and moves it to a designated **trash location** before releasing it.

The full stack runs on **Ubuntu 22.04 + ROS 2 Humble**, combining perception, motion planning, and hardware actuation into a single closed-loop pipeline.

### Demo Video (Closed-Loop Cycle)

> **Demo video:** [Obstacles Full Run](https://youtu.be/Z1cQZPDRZZg)  
> This clip shows one full operation cycle:
> 1. The arm starts from a home pose while the pole-mounted camera observes the workspace.  
> 2. The leaf detection node identifies leaf positions and their health status.  
> 3. The automation node selects a target leaf and MoveIt plans a trajectory.  
> 4. The UR5e moves to the treatment pose: either spraying pesticide on a healthy leaf or gripping an unhealthy leaf with the vacuum and placing it into the trash area. All while avoiding obstacles(blue boxes). 
> 5. RViz visualises the robot, detected leaves, and pump status in real time as the arm returns home, ready for the next leaf.

# System Architecture 

At a high level, the system is organised into ROS 2 packages that handle robot description and transforms, leaf perception, motion planning, task automation, hardware actuation, and monitoring. These components communicate via standard ROS topics, TF frames, and custom services to form a complete perception–planning–actuation loop.

### Node and Communication Graph

The diagram below shows the main nodes and their connections. It captures the data flow from the depth camera, through the leaf detection server and task automation node, to the UR5e and the Arduino-controlled sprayer/vacuum tool.


**Figure 1 – Node and communication graph for the pesticide sprayer arm**  
![mtrn 4231 group](https://github.com/user-attachments/assets/773d51d1-f6c6-4f4a-a595-bdcb52bcfb65)

Key components in the graph:

- **Perception**
  - **Camera driver** (e.g. RealSense node)
    - Publishes RGB and depth images:
      - `/camera/camera/color/image_raw`
      - `/camera/camera/depth/image_rect_raw`
    - Publishes camera info:
      - `/camera/camera/color/camera_info`
      - `/camera/camera/depth/camera_info`
  - **Leaf detection server** (`leaf_detection_server`, in `detect_leaf_pkg`)
    - Subscribes to RGB and depth topics from the pole-mounted camera.
    - Uses camera intrinsics to recover 3D points.
    - Uses TF to transform points from the camera frame into the robot `base_link` frame.
    - Provides a custom service:
      - `/leaf_detection_srv` (`arm_msgs/srv/LeafDetectionSrv`)
        - Request: detection command and parameters.
        - Response: array of leaf coordinates, success flag, debug message.
  - **Leaf visualisation node** (`leaf_visualization_node`, in `detect_leaf_pkg`)
    - Subscribes to detection outputs.
    - Publishes `visualization_msgs/Marker` / `MarkerArray` to RViz to show leaf positions and auxiliary features (e.g. workspace box).

- **Manipulation and Robot Control**
  - **UR5e driver node** (e.g. `ur_robot_driver`)
    - Interfaces with the physical UR5e.
    - Publishes `/joint_states` and accepts joint trajectory commands.
  - **Robot state publisher / TF tree**
    - Publishes transforms between `base_link`, arm links, end-effector, and the pole-mounted camera.
  - **MoveIt `move_group` node**
    - Performs motion planning for the UR5e.
    - Interacts with the planning scene (static and dynamic collision objects).
    - Accepts planning requests from helper nodes.
  - **Move-to-pose node** (`move_arm_to_pose`, in `arm_manipulation`)
    - Uses MoveIt’s C++ API to:
      - Plan to target poses in `base_link` frame.
      - Execute trajectories on the UR5e.
    - Target positions are provided via parameters or launch arguments.
  - **Dynamic obstacle monitor** (`dynamic_obstacle_control`, in `dynamic_obstacles_monitor`)
    - Subscribes to a topic such as `obsFromImg` containing `moveit_msgs/msg/CollisionObject`.
    - Adds/removes collision objects in the MoveIt planning scene to represent dynamic obstacles.

- **Task Automation / High-Level Control**
  - **Automation orchestrator** (`automation_orchestrator`, in `task_automation`)
    - Calls `/leaf_detection_srv` to obtain leaf positions and (configurable) health classification.
    - For each selected leaf:
      - Computes a safe treatment pose in `base_link` frame.
      - Calls the motion component (`move_arm_to_pose` / MoveIt) to move the UR5e.
      - Calls the Arduino service to actuate either the sprayer (for healthy leaves) or the vacuum gripper (for unhealthy leaves).
      - Optionally moves to a trash pose to discard removed leaves.
    - Manages parameters such as home pose, trash pose, Z-limits, biases, and timeouts.

- **Hardware Actuation**
  - **Arduino communication node** (`leafServerNode`, in `arduinoCommunication`)
    - Opens a serial connection to the Arduino controlling pumps.
    - Provides the service:
      - `/send_command` (`arduinoCommunication/srv/LeafCommand`)
        - Accepts commands to enable/disable sprayer and vacuum separately.
    - Publishes a `visualization_msgs/Marker` on `pump_status_marker` to show the current pump state in RViz.

- **Monitoring and Visualisation**
  - **Arm monitoring node** (`arm_position_viewer`, in `arm_monitoring`)
    - Reads TF and/or joint states.
    - Publishes a marker or text overlay with the current end-effector position in `base_link`.
  - **RViz**
    - Displays:
      - UR5e model and TF tree,
      - Leaf markers,
      - Pump status marker,
      - Arm position marker,
      - Planning scene obstacles and trajectories.

Together, these nodes form a loop: camera → leaf detection → automation → motion → actuation, with RViz providing continuous visual feedback.

### Package-Level Architecture

The workspace is structured into ROS 2 packages, each with a clear responsibility:

| Package                     | Main Nodes / Components                 | Responsibility                                                                 |
|----------------------------|-----------------------------------------|---------------------------------------------------------------------------------|
| `robot_description`        | URDF/xacro, RViz configs, TF setup      | Defines the UR5e, pole-mounted camera, and end-effector, and provides the TF tree. |
| `detect_leaf_pkg`          | `leaf_detection_server`, visualisation  | Performs leaf detection and 3D localisation from RGB-D data and publishes RViz markers. |
| `arm_msgs`                 | `LeafDetectionSrv`                      | Defines the shared service interface for leaf detection results.               |
| `arduinoCommunication`     | `leafServerNode`, LeafCommand srv       | Bridges ROS to Arduino over serial and exposes sprayer/vacuum control as a ROS service. |
| `arm_manipulation`         | `move_arm_to_pose`, collision objects   | Uses MoveIt to plan and execute UR5e trajectories and defines static collision geometry. |
| `arm_monitoring`           | `arm_position_viewer`                   | Publishes visual markers/text overlays of the arm’s current position.          |
| `task_automation`          | `automation_orchestrator`               | Implements the high-level task logic for spraying healthy leaves and removing unhealthy leaves to trash. |
| `default_scripts` (folder) | Shell scripts, helper launch commands   | Provides convenience scripts for workspace setup, bringup, and automation runs. |

This modular structure allows each component (perception, manipulation, actuation, automation) to be developed and tested individually while still composing into a single integrated system.

### Task Logic and State Machine

The high-level behaviour of the system is implemented in the `automation_orchestrator` node as a state-machine-like sequence. Conceptually, the task can be described with the following states:

> **Figure 2 – High-level task state machine Flowchart**
<img width="4375" height="593" alt="Flowchart" src="https://github.com/user-attachments/assets/dcc5756a-ec42-4966-8029-386634167675" />

**States:**

1. **Idle / Initialise**
   - Load parameters (home pose, trash pose, biases, Z-limits, timeouts).
   - Wait for required services and topics:
     - `/leaf_detection_srv` (leaf detection),
     - `/send_command` (Arduino),
     - MoveIt / robot bringup.

2. **DetectLeaves**
   - Call `/leaf_detection_srv` with configured parameters (e.g. minimum area, confidence).
   - Receive a set of leaf positions in `base_link` frame, and optionally a health label (healthy vs unhealthy).
   - If no leaves are detected within a timeout, either retry or transition to **Finished**.

3. **SelectTargetLeaf**
   - Choose the next leaf to process from the detection list (e.g. nearest, left-to-right, or simply in order).
   - Determine whether the leaf is **healthy** (to be sprayed) or **unhealthy** (to be removed to trash).

4. **PlanAndMoveToLeaf**
   - Compute a treatment pose above the leaf position:
     - Apply XY bias and clamp Z between configured `z_min` and `z_max`.
   - Request a collision-free trajectory from MoveIt (via `move_arm_to_pose` or directly through MoveIt APIs).
   - Execute the trajectory on the UR5e.
   - On planning or execution failure (timeout, collision, unreachable pose), either retry with adjusted parameters or skip to the next leaf.

5. **ActuateTool**
   - If the target leaf is **healthy**:
     - Call `/send_command` to enable the **sprayer** (and disable vacuum).
     - Hold for a configured duration, then stop spraying.
   - If the target leaf is **unhealthy**:
     - Call `/send_command` to enable the **vacuum gripper** (and disable sprayer) to grip the leaf.
     - After gripping, transition to **MoveToTrash**.

6. **MoveToTrash** (for unhealthy leaves)
   - Plan and execute a motion from the current pose to the configured **trash pose**.
   - Once at the trash location, call `/send_command` to release the leaf (disable vacuum).
   - Optionally dwell for a short time to ensure the leaf has dropped.

7. **ReturnHome**
   - Plan and execute a motion back to the configured **home pose**.
   - This provides a consistent reset before processing the next leaf.

8. **NextLeafOrFinish**
   - If there are remaining leaves in the current detection set:
     - Transition back to **SelectTargetLeaf**.
   - Otherwise:
     - Optionally call **DetectLeaves** again to refresh the scene.
     - If no new leaves are found or a maximum number of cycles is reached, transition to **Finished**.

9. **Finished / Error**
   - Stop actuation and leave the arm in a safe pose (e.g. home).
   - Log summary information (number of healthy leaves sprayed, number of unhealthy leaves removed, failures/timeouts).

This state-machine structure makes it clear how perception, planning, and actuation are coordinated and provides a natural basis for future extensions (e.g. priority ordering of leaves, more advanced health classification, or replacement with a formal behaviour tree).

# Technical Components
## Computer Vision
1. RGB-D Camera
  * The RGB frame provides pixel-level colour information needed to classify the leaves.
  * The depth frame provides distance values for each pixel, allowing the system to compute real-world 3D coordinates of each leaf.
2. Leaf Detection
  * The RGB image is processed to detect leaves lying flat on the table. The pipeline currently identifies damaged leaves using a yellow cross marker placed on them. 
3. Pixel Coordinate Extraction
  * For each detected leaf, the centroid and bounding box coordinates are extracted from the RGB image.
  * These pixel coordinates represent the reference point for depth lookup.
4. Depth Extraction and 3D Reconstruction
  * The corresponding depth value at the leaf’s pixel location is sampled from the depth frame.
  * Combined with camera intrinsics, these values are used to compute the leaf’s 3D position in the camera coordinate frame.
  * This process converts 2D detections into metric-space (x, y, z) coordinates.
5. Transformation to Robot Coordinate Frame
  * The system applies the precomputed hand–eye calibration transform to map 3D leaf positions from the camera frame into the robot’s base frame.
  * These transformed coordinates are then forwarded to the motion planner, for vacuuming or spraying.

**Role of Vision Pipeline**
The vision pipeline allows for automatic leaf detetction, removing the need for manual inspection. It classifies leaf health directly from visual cues. It computes 3D leaf positions using depth data and ensures that the robot's behaviour is directly driven by visual data. It enables the system to adapt to different lead locations each run, supporting closed-loop operation during detection. 

## Custom End-Effector
### Photos/renders
|Component        |STL File                                                                   |
|-----------------|---------------------------------------------------------------------------|
|Whole Assembly   |[Whole Assembly STL](CustomEndEffector/STL%20Files/FullAssembly.stl)       |
|Closing Mount    |[Closing Mount STL](CustomEndEffector/STL%20Files/ClosingMount.stl)        |
|Vacuum Pump Mount|[Vacuum Pump Mount STL](CustomEndEffector/STL%20Files/VacuumPumpMount.stl) | 
|Spray Pump Mount |[Spray Pump Mount STL](CustomEndEffector/STL%20Files/SprayMountBox.stl)    |
|Provided Mount   |[Provided Mount STL](CustomEndEffector/STL%20Files/ProvidedMount.stl)      |
|End Effector     |[End Effector STL](CustomEndEffector/STL%20Files/EndEffectorComponent.stl) |

### Assembly details
- All Components were modelled in Fusion360 and printed using a Creality Ender V3 3D printer with black PLA filament.
- All components are push-fit, meaning that no tape, glue or expoxy is used.
- The spray pump is screwed onto the Spray Pump Mount using 2 x M3 x 10mm screws.
- The vaccumm pump is threaded into the M10 x 1.0 hole on the end effector body.
- A MOSFET is placed on the rear of the end-effector using double sided foam tape, to control the spray pump and prevent high currents from entering the Arduino.
- Wires are soldered and insluated (electrical taped) to prevent accidental disconnections.
  
### Engineering drawings
|Component         |Drawing                                                                          |
|------------------|---------------------------------------------------------------------------------|
|Whole Assembly    |[Whole Assembly](CustomEndEffector/Drawings/FullAssemblyDrawing.pdf)             |
|Closing Mount     |[Closing Mount](CustomEndEffector/Drawings/ClosingMount.pdf)                     |
|Vacuum Pump Mount |[Vacuum Pump Mount](CustomEndEffector/Drawings/VacuumPumpMount.pdf)              |
|Spray Pump Mount  |[Spray Pump Mount](CustomEndEffector/Drawings/SprayMountBox.pdf)                 |
|Provided Mount    |[Provided Mount](CustomEndEffector/Drawings/ProvidedMount.pdf)                   |
|End Effector      |[End Effector Mount](CustomEndEffector/Drawings/EndEffectorComponentDrawing.pdf) |

### Control overview 
- The system uses ROS2, Python and an Arduino UNO to coordinate spraying and leaf-picking operations.
- The camera detects incoming leaves and classifies their condition. This classification determines the action sequence: pesticide spraying or vacuum leaf removal.
- Once a leaf is detected, the controller computes the 3D location and sends a motion command to the UR5e using the calibrated DH parameters. 
- If spraying is required, the ROS2 Node sends a request to the Arduino via a client-server interface, activating a 12V pump through the MOSFET driver to deliver a controlled spray.
- If the leaf needs removal, the robot positions the vacuum nozzle above the leaf, lowers until contact is made, activates suction, and transfers the leaf to the disposal bin.
  
### Integration details.
* The end-effector was designed to remain lightweight, minimise inertia and avoid excessive torque at the wrist joints of the UR5e.
* Mounting geometry aligns with the Provided Mount, ensuring capability with the UR5e arm.
* Cable routing is kept to the rear side of the tool to avoid entanglement during wrist rotation.
* Electrical components are positioned to remain within the robot's collision-free zone.
* The ROS2-Arduino interface allows asynchronous communication, enabling:
  * Spray duration control
  * Vacuum on/off control
* The modular design allows components to be easily swapped without removing the whole end effector.
  
## System Visualisation
RViz2 is used to visualise the robot, camera data, detected leaves and obstacles, allowing for verification of the perception and manipulation pipeline. 
**Robot Model**
The UR5e robot model is displayed with live joint states. This shows the robot's real time configuration and confirms that planned motions match actual robot movement. 
**Safety Planes** 
Safety planes are constructed and displayed, to restrict the robot's planned trajectories. A ceiling, floor (table), and side walls are constructed to ensure the robot arm does not move beyond this planes and remain in the workspace. 
**Obstacles**
Obstacles are displayed as a green box, with its dimension and position updating in real time, based on the camera data. This allows for visual confirmation of the planner and robot being aware of obstacles and avoiding them during motion planning. 
**Leaf Detection Markers**
Detected leaves are shown as visualisation markers:
* Green for healthy leaves
* Yellow for unhealthy leaves.
These markers update in real-time and appear at the 3D positions used by the robot.
**Projected Robot Arm Configuration**
The projected arm configuration is displayed as an orange UR5e arm in RViz2. This provides visual confirmation of the robot moving to the correct position.

> **Figure 3 – Rviz2 Visualisation**
<img width="935" height="445" alt="Screenshot from 2025-12-03 17-04-21" src="https://github.com/user-attachments/assets/f6d96e09-9262-4ef1-a132-aa1c33001677" />


## Closed-Loop Operation
The system used a closed-loop approach during the detection and task-planing phase to ensure the robot's actions are based on real-time information from the environment. 
The RGB-D camera continuously provides RGB data for classification of healthy vs unhealthy leaves, depth data for 3D locations of each leaf, and updated centroid positions whenever the perception node reprocesses the scene. 
Before the robot begins a vacuum or spray action, the vision pipeline refreshes sll detetctions and recomputes their 3D coordinates. These updated positions are then forwarded to the robot. 

**System Adaptation in Real Time**
- If a leaf/ new leaf/ obstacle appears before the robot starts its motion, the updated detection changes the target coordinates.
- This ensures the robot always moves to the most recent leaf position.
- The robot then plans a trajectory based on this updated feedback, ensuring accurate alignment for picking or spraying. 

This system is closed-loop during perception and decision making. Every task starts with a fresh detection cycle using real-time camera data. Once motion execution begins, the robot follows the planned trajectory. This feedback approach:
- Minimises localisation errors in leaf and obstacle positions.
- Ensures that classification and localisation stay up to date.
- Allows the robot to adapt to the environment at the start of each operation.
- Improves reliability and accuracy across vacuum and spraying tasks, preventing outdated imformation from influencing motion planning.


## Installation and Setup

1. Hardware Setup Information
  * UR5e robot
       * Ensure the UR5e is **powered on** and the **ROS program** is running on the teach pendant. 
       * Place the robot in its home position before launching the system.
       * Confirm the robot is connect to the ROS2 network (via Ethernet)
  * RGB-D Camera
       * Use the provided table-mounted RGB-D camera in the setup.
       * The camera streams:
           * RGB images for leaf and obstacle classifications.
           * Depth frams for 3D localisation.
       * Ensure the camera is securely mounted, movement of the camera will invalidate calibration.
  * Arduino UNO
       * Controls vacuum and spray motors.
       * Communicates with the robot via UART and a customn ROS2 Client/Server.
       * The provided Arduino code must be uploaded to the board prior to operation
    
2. Environment Variables and Configuration files
  **ROS2 Environment**
  * Source ROS2 before launching
  ```bash
  source install/setup.bash
  ```
  * Ensure required packages are built
  ```bash
  colcon build
  ```
  **Calibration Requirements**
  * A precomputed transform between the camera and the robot base must exist (This system assumes this file is already provided)
  * Z-Axis Offset Calibration: Must adjust the z-axis offset for differences in table height, leaf thickness, mounting variations. This ensures precise spraying and prevents the vacuum from making excessive contact. 
  * HSV Threshold Calibration: Lighting conditions vary, and HSV values must be adjusted for reliable detection.
  * Visual Markers: During operation, ensure that yellow visual markers represent unhealthy leaves and green markers represent healthy leaves. 

3. Software Installation & Workspace Setup
Clone the workspace
```bash
git clone https://github.com/darshan-k-s/leaf-sorting-sprayer-arm.git
cd leaf-sorting-sprayer-arm
```

Install dependencies
```bash
sudo apt update
rosdep install --from-paths src -y --ignore-src
pip install -r requirements.txt
```

Build the ROS2 packages
```bash
colcon build
source install/setup.bash
```
   
## Running the System 

This section describes how to bring up all core nodes and run the closed-loop automation task.

### Real robot
We don't need to build the workspace and do the sourcing. There are scripts in 'default_scripts/' we developed that automatically handle workspace building and environment setup. 
But before that, let's go through hardware checks and procedures.
- Power on the UR5e and release any safety stops.
- Ensure the controller is in Manual / Automatic control mode(all wired connections to our computer are assumed).
- Confirm you can ping the robot:
```bash
ping 192.168.0.100
```
Then go inside the repository folder.
```bash
cd <workspace_root>
# One-time: ensure scripts are executable
chmod +x default_scripts/*.sh
```
After this, only one script starts the whole system with the real hardware. The automation is a different script.
```bash
# Start full system on real hardware
./default_scripts/start_all_real.sh
```
This will spawn multiple terminal windows, each with a different set of nodes:
- DriverServer – UR5e driver node
- MoveitServer – MoveIt + RViz2
- CollisionObjects – static collision objects for the planning scene
- ArmMonitoring – arm position viewer
- RobotCameraTF – static transforms between robot base and camera
- Camera – RealSense camera node
- ObstacleMonitor – dynamic obstacle monitor (if used)
- LeafDetection – leaf detection server + visualisation
- ArduinoServer – Arduino communication node for sprayer/vacuum

Wait until all terminals open up and RViz2 pops up. It should have a pre-loaded MoveIt2 scene with the robot, collision planes, detected leaf markers, the attached end-effector and the camera. This is the fully loaded robot scene, ready to take movement commands with real-time obstacle detection and avoidance. 
The camera feed topic can be added to Rviz for better visualisation of the table(shows type and detected leaves). 

Now we look at launching the automation.
```bash
./default_scripts/run_automation.sh
```
This spins up the `automation_orchestrator`. The orchestrator calls `/leaf_detection_srv` first. The leaf detection server processes the latest RGB-D frame from the pole-mounted camera, segments leaves, and outputs 3D positions in the base_link frame. 
For each detected leaf, the orchestrator selects a target and computes a treatment pose above the leaf(applying XY bias and clamping Z between `z_min` and `z_max`). MoveIt plans a collision-free trajectory and executes it on the UR5e. 
At the treatment pose, for healthy leaves, the sprayer is activated via `/send_command` for a configured duration. For unhealthy leaves, the vacuum gripper is activated to grip the leaf, then the arm moves to the trash pose and releases it.
The orchestrator then moves on to the next leaf, or goes to home position at the end of the automation task.


### Simulation
We have no simulator used in the project (like Gazebo), rather, we mimic the UR5e interface through code and use Rviz2 for visualisation. The workflow is similar to the real robot bringup. This allows us to perform Hardware in the Loop(HITL) visualisations. We can attach the real hardware, along with the camera, or use a `rosbag` or pre-recorded video feed to publish to the camera topic. 
Only one script starts the whole simulation environment. The automation is a different script.
```bash
# Start full system on fake hardware
./default_scripts/start_all.sh
```
This will spawn multiple terminal windows, each with a different set of nodes, as before.
Wait until all terminals open up and RViz2 pops up. It should have a pre-loaded MoveIt2 scene with the robot, collision planes, detected leaf markers, the attached end-effector and the camera. This is the fully loaded robot scene, ready to take movement commands with real-time obstacle detection and avoidance. 
Now we launch the automation the same way.
```bash
./default_scripts/run_automation.sh
```
This would show the robot's movement precisely in RViz2.

### Troubleshooting

Some common issues and checks:
##### Abnormal leaf detections
- Check that the camera node is running:
```bash
ros2 topic list | grep camera
```
- Then verify that the pole-mounted camera actually sees the plants(no occlusions).
- Check lighting conditions; strong glare or very low light can break segmentation. In this case we'll need to tweak the HSV values for the contemporary lighting. Use `standalone_leaf_detection.py` to choose a set of values in real-time. Note them down and then change the respective values in the HSV parameters defined in `/detect_leaf_pkg/detection_handler.py`. 

##### Abnormal blue box detections
- The detection of blue box obstacles have the same flow as the leaves.
- Make sure parts of the robot aren't detected as obstacles either (this will halt motion planning). 
- Then try tweaking the HSV values for proper detection through `adjust_blue_box_thresholds.py`, note the values and change the respective values in the HSV parameters defined in `/detect_leaf_pkg/detection_handler.py`.  

##### Automation script fails to start
- Make sure the workspace is built and sourced. That is, check if you've already run `start_all.sh` or `start_all_real.sh`, which bringup the robot. 
- Then, verify whether required services exist:
```bash
ros2 service list | grep leaf_detection_srv
ros2 service list | grep send_command
```
If they don't show up, you'll have to bringup the robot again.

##### MoveIt cannot find a plan / motion aborted
- Check that the static collision objects (table, walls) are correctly loaded.
- Then make sure the leaf positions are within the reachable workspace of the UR5e.
- Adjust Z limits (z_min, z_max) or XY biases if the end-effector is too close to the table or too far from the leaves.
- Check for false or abnormal leaf and blue box obstacles detections. Also, sometimes when the arm is covering a blue box, the box as an obstacle might be blown up in height due to limitations of the camera and it's positioning.

##### Arduino / pump issues
- Confirm the Arduino node is running in the ArduinoServer terminal and connected to the correct serial port.
- If `/send_command` is missing, restart the ArduinoServer terminal or rerun `start_all_real.sh` or `start_all.sh`.
- Check physical wiring, power supply, and that the pumps/vacuum are correctly connected.
- Check if Arduino drivers are installed on your system. Installing the Arduino IDE would do this for you by default. 
- Lastly, test the working through the Serial Monitor of the Arduino IDE. This will require knowledge of pin outs and connections.

##### RealSense camera not detected
- Run the dedicated camera script:
```bash
cd <workspace_root>
./default_scripts/camera.sh
```
- Ensure the camera appears in `lsusb`.
- Try a different USB port or cable if the device is not detected.


If problems persist, check the logs in each terminal window (DriverServer, MoveitServer, LeafDetection, ArduinoServer, etc.) for error messages, and verify that all nodes appear in:
```bash
ros2 node list
ros2 service list
ros2 topic list
```


## Results and Demonstration

> **[Full run with obstacles](https://youtu.be/Z1cQZPDRZZg)**  
> (Short clip of the full cycle: detect leaf → move → spray/vacuum → return home.)

> **[Full run without obstacles](https://youtu.be/ZFiR_eWnI4c)**  
> (Short clip of the full cycle: detect leaf → move → spray/vacuum → return home.)

#### 1. System Performance
- The robot successfully detects and classifies healthy and unhealthy leaves in real time using OpenCV pipeline
- Damaged/bad leaves are accurately picked up and removed, while healthy leaves are sprayed with minimal error
- The system adapts to minor changes in leaf positions due to its closed loop operation.
#### 2. Quantitative Results
- **Detection Accuracy:** ~97% on the test set of leaves.
- **Pick-up Repeatability:** Leaves consistently grasped within +/- 10 mm of target position.
- **Spray Precision:** Healthy leaves sprayed successfully in ~ 100% of attempts.
#### 3. Demonstration
- Visualisation in RViz2 shows live leaf detection, robot trajectories, and end-effector state.  
- Photos, CAD renders, and short demonstration videos illustrate the full pick-and-spray operation.  
- Yellow crosses mark leaves detected as unhealthy for easy verification.  
#### 4. Highlights
**Robustness:** 
The system maintains reliable performance despite environmental noises and lighting variation. Real-time HSV tuning, error checking and motion compensation make the pipeline resilient during operation. 
**Adaptability:**
The robot dynamically adjusts its trajectory based on live detections, enabling it to handle changing leaf orientations, plant shapes, and workspace conditions. This closed-loop design avoids hard-coded assumptions and works effectively in unstructured environments.  
**Innovation:** 
The dual-function end effector, live perception-driven control, and modular hardware architecture represent a novel combination not commonly found in similar robotic plant-care systems. The ability to switch between vacuum-based removal and precision spraying using a single tool is unique.


## Discussion and Future Work
**Reliable Leaf Detection**
* The change in environment introduced noise in segmentation. Adjusting HSV thresholds imrpoved stability, but performance still degrades under variable lighting. 
**Stable robot-camera calibration**
* There were issues with the IntelSense camera, where markers were detected at the wrong location. Resolving this issue involved reconnecting the camera. In doing do, reconnecting the camera could shift extrinsics, causing markers to appear in the wrong place in RViz2. Future work should include a calibration routine that enables the markers to be detected in the correct position. 
**End Effector airflow and spray consistency**
* Achieving precise spraying without disturbing surrounding leaves required tuning the z-axis offset and regulating airflow pressure.

**Opportunities for improvement**
  * Integrating a RGB-D camera on the end effector would significantly improve disease detection and 3D localisation from close range.
  * Better disease classification: integrate a more detailed plant model to classify leaf health more accurately (yellow chlorosis, fungal infection)
  * Full Robot Workspace Mapping: using a structured-light scanning to model the whole plant geometry for improved motion planning
    
**Extensions:**
* Combining the vacuum with a mciro-gripper to 'pluck' leaves from a tree branch.
* Conducting the project on a vertical branch, with leave sticking out rather than flat on the table.
* Having multiple spray pumps for different pesticides, ensuring targeted treatment.
* Having a tool changing mechanism, where vacuum pump and spray nozzle interchange.

**Novelty and Effectiveness**
  * Fully closed-loop operation: The robot dynamically adapts its motion to real-time leaf detections rather than relying on hard-coded positions.
  * Dual function end-effector: A single tool head performs both leaf removal and precise spraying, reducing hardware complexity whilst increasing versatility.
  * Low hardware cost and modular design: Components such as motor mounts and airflow modules cna be swapped or upgraded easily, enabling reuse across applications. 
  * High flexibility: The combination of vision-based detection and adaptive control allows the system to handle variable leaf sizes, orientation and plant layouts. 

## Contributors and Roles
|Team Member            |Primary Responsibilities|
|-----------------------|------------------------|
|Hao Yu                 | Led computer vision development (Leaf detection and depth processing) <br> Designed and implemented obstacle avoidance path planning <br> Integrated all software and hardware components into final working system|
|Darshan Komala Sreeramu| Developed robust path planning algorithms <br> Added safety planes and robot joint constraints to ensure safe robot motion <br> Contributed to obstacle-avoidance planning and motion-control refinement|
|Daniel Bui             | Responsible for hardware assembly and electronics <br> Designed and 3D printed the custom end-effector for leaf pick-up and spraying|


## Repository Structure

```text
Hao_MTRN4231/
├── default_scripts/              # Shell scripts for bringup / network / camera
│   ├── start_all.sh             # One-command bringup (fake UR5e + MoveIt)
│   ├── start_all_real.sh        # One-command bringup for real hardware
│   ├── run_automation.sh        # Start the automation task orchestrator
│   ├── setupFakeur5e.sh         # Fake UR5e + MoveIt bringup
│   ├── setupRealur5e.sh         # Real UR5e bringup
│   ├── camera.sh                # RealSense camera node
│
├── python_scripts/               # Helper/tuning scripts (not ROS packages)
│   ├── adjust_leaf_thresholds.py     # Interactive tuning of leaf detection HSV / area thresholds
│   ├── adjust_blue_box_thresholds.py # Tuning blue box detection parameters
│   ├── check_arduino.py              # Check Arduino serial devices
│   └── standalone_leaf_detection.py  # Standalone leaf detection test
│
├── src/                          # ROS2 workspace source (all ROS packages)
│   ├── arduino_communication/    # Arduino communication (vacuum/spray service)
│   ├── arm_manipulation/         # UR5e + MoveIt planning and collision scene (incl. dynamic_obstacle_control)
│   ├── arm_monitoring/           # Arm state monitoring/visualization (arm_position_viewer)
│   ├── arm_msgs/                 # Custom service interfaces (LeafDetectionSrv)
│   ├── detect_leaf_pkg/          # Leaf detection and visualization (RGB-D + PlantCV + TF)
│   ├── robot_description/        # Robot and camera URDF / Xacro / extrinsic calibration
│   └── task_automation/          # Automation orchestration (calls detection + arm + Arduino)
│
├── CustomEndEffector/            # Constains STL files and drawings
│   ├── Drawings/                 # Engineering drawings of all components
│   ├── STL Files/                # STL Files of all components
│
├── requirements.txt              # Python dependencies (pip: numpy / opencv-python / plantcv / pyrealsense2)
└── README

```

## References and Acknowledgements

This project was completed as part of **MTRN4231 – Robotics and Autonomous Systems** at UNSW Sydney.  
We would like to thank:

- The **MTRN4231 course convenor and teaching staff** for guidance on ROS 2, MoveIt, and project requirements.
- The **lab demonstrators and technical staff** for their support with the UR5e, camera setup, and hardware troubleshooting.
- Our fellow students in the robotics lab for discussions, shared debugging sessions, and feedback on system design.
- The maintainers of the open-source ROS, MoveIt and RealSense ecosystems, whose tools and examples formed the backbone of this project.


The following resources were particularly useful when developing this system:

- **ROS 2 Humble documentation** – for node composition, parameters, launch files, and TF2 usage.
- **MoveIt 2 Tutorials** – for setting up the UR5e MoveIt configuration, planning scene, and `MoveGroupInterface`-based motion planning.
- **Universal Robots ROS 2 driver** – for integrating the physical UR5e with ROS 2 and enabling external control.
- **Intel RealSense SDK and ROS wrapper** – for configuring the pole-mounted RGB-D camera and accessing synchronised colour and depth streams.
- **PlantCV** – for implementing leaf segmentation and basic health classification from RGB imagery.
- **RViz 2** – for visualising the robot model, TF frames, leaf markers, pump status, and planning scene collision objects.
