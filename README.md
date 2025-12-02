# Leaf Sorting and Sprayer Arm

This repository contains the ROS 2 Humble workspace for a UR5e-based leaf sorting and sprayer arm.
The system uses a pole-mounted Intel RealSense camera and a custom sprayer/vacuum end-effector to detect leaves in 3D, plan collision-free trajectories with MoveIt, and pick-and-place healthy leaves into a box or actuate a pump via an Arduino bridge to spray pesticide on diseased leaves – all in a closed-loop pipeline.

The project was developed and tested on **Ubuntu 22.04 + ROS 2 Humble** with a **UR5e**, **RealSense RGB-D camera**, and a **custom built pesticide sprayer/vacuum end-effector**.

> **Demo video:** _[link coming soon]_  
> (Short clip of the full cycle: detect leaf → move → spray/vacuum → return home.)

---

## Table of Contents(Update on the go)

- [Leaf Sorting and Sprayer Arm](#leaf-sorting-and-sprayer-arm)
  - [Table of Contents(Update on the go)](#table-of-contentsupdate-on-the-go)
- [Project Overview (Hayden)](#project-overview-hayden)
- [System Architecture (Hayden)](#system-architecture-hayden)
- [Technical Components (Hayden + Daniel)](#technical-components-hayden--daniel)
  - [Computer Vision](#computer-vision)
  - [Custom End-Effector](#custom-end-effector)
  - [System Visualisation](#system-visualisation)
  - [Closed-Loop Operation](#closed-loop-operation)
  - [Installation and Setup (Hayden + Daniel)](#installation-and-setup-hayden--daniel)
  - [Running the System (Darshan)](#running-the-system-darshan)
  - [Results and Demonstration](#results-and-demonstration)
  - [Discussion and Future Work](#discussion-and-future-work)
  - [Contributors and Roles](#contributors-and-roles)
  - [Repository Structure](#repository-structure)
  - [References and Acknowledgements](#references-and-acknowledgements)



# Project Overview (Hayden)

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

> **Demo video:** _[link to 10–30s video]_  
> This clip shows one full operation cycle:
> 1. The arm starts from a home pose while the pole-mounted camera observes the workspace.  
> 2. The leaf detection node identifies leaf positions and their health status.  
> 3. The automation node selects a target leaf and MoveIt plans a trajectory.  
> 4. The UR5e moves to the treatment pose: either spraying pesticide on a healthy leaf or gripping an unhealthy leaf with the vacuum and placing it into the trash area.  
> 5. RViz visualises the robot, detected leaves, and pump status in real time as the arm returns home, ready for the next leaf.



[
- short description of task/problem your system solves, including intended "customer" or end-user
- summary of robot's functionality
- short video (10-30s) of the robot completing one full cycle/operation demonstrating closed-loop behaviour and visualisation (embedded or external link to Youtube/OneDrive/Google Drive video)

]


# System Architecture (Hayden)

At a high level, the system is organised into ROS 2 packages that handle robot description and transforms, leaf perception, motion planning, task automation, hardware actuation, and monitoring. These components communicate via standard ROS topics, TF frames, and custom services to form a complete perception–planning–actuation loop.

### Node and Communication Graph

The diagram below (to be added) shows the main nodes and their connections. It captures the data flow from the pole-mounted camera, through the leaf detection server and task automation node, to the UR5e and the Arduino-controlled sprayer/vacuum tool.

> **Figure 1 – Node and communication graph for the pesticide sprayer arm**  
> _(Insert rqt_graph / custom diagram here.)_

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
| `dynamic_obstacles_monitor`| `dynamic_obstacle_control`              | Injects dynamic collision objects into the MoveIt planning scene at runtime.   |
| `arm_monitoring`           | `arm_position_viewer`                   | Publishes visual markers/text overlays of the arm’s current position.          |
| `task_automation`          | `automation_orchestrator`               | Implements the high-level task logic for spraying healthy leaves and removing unhealthy leaves to trash. |
| `default_scripts` (folder) | Shell scripts, helper launch commands   | Provides convenience scripts for workspace setup, bringup, and automation runs. |

This modular structure allows each component (perception, manipulation, actuation, automation) to be developed and tested individually while still composing into a single integrated system.

### Task Logic and State Machine

The high-level behaviour of the system is implemented in the `automation_orchestrator` node as a state-machine-like sequence. Conceptually, the task can be described with the following states:

> **Figure 2 – High-level task state machine**  
> _(Insert state machine diagram here.)_

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







[
- diagram of ROS2 nodes, topics, services and actions (from rqt_graph or custom schematic)
- package-level architecture diagram showing node interactions and topics
- behvaiour tree or state-machine diagram showing closed loop system behaviour
- brief description of the function of each node
- any custom message types or interfaces should be listed and explained. 

]


# Technical Components (Hayden + Daniel)
## Computer Vision
describe vision pipeline and how it contributes to the task.
## Custom End-Effector
### Photos/renders
- provide links STL files
### Assembly details
- for this project, the components were 3D printed using Creality Ender V3 3D printer.
- all components are push-fit, meanign that no tape/adhesive is used
- the spray mount is screwed onto the (SprayMount) using 2 x M3x1.5 10mm screws
- the vaccumm pump is threaded into the M10 x 1.0 hole on the end effector component
- A mosfet is used to control the spray pump, preventing high currents from entering the Arduino, protecting it
### Engineering drawings
- provide links to PDFS 
### Control overview 
- the system uses ROS2 and Arduino to coordinate spraying and leaf-picking operations
- the camera provided detects incoming leaves and classifies them based on .... This classification determines the action sequence
- once leaf is detected, controller moves robotic arm to appropriate position using DH parameters
- if the leaf needs treatment, the system communicated with the Arduino over server/client communication and activates a 12V pump through a MOSFET driver to spray a controlled amount of pesticide.
- If the leaf needs to be removed, the arm lowers the vacuum to pick it up and transfer it to the designated bin
- 
### Integration details.
## System Visualisation
explain how system is visualised (RViz) and what it demonstrates
## Closed-Loop Operation
describe the feedback method and how it adapts system behaviour in real time

## Installation and Setup (Hayden + Daniel)
- step-by-step installation instructions for dependencies and workspace setup
- hardware setup information (UR5e connection, camera, Teensy, etc.)
  * camera is mounted on the provided bracket
  * Arduino is used to code the motor configuration
  * communication is done over UART and ROS2 Client/Server communication
- any environment variables, configuration files, or calibration prcedures required to run the system (can assume there is some sort of hand-eye calibration already present in the system)
  * having yellow crosses on the leafs provide a clear indication of bad/good leaves

## Running the System (Darshan)

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
After this, only one scriptws starts the whole system with the real hardware. The automation is a different script.
```bash
# Start full system on real hardware
./default_scripts/start_all_real.sh
```
This will spawn multiple terminal windows, each with a different set of nodes:
- DriverServer – UR5e driver node
- MoveitServer – MoveIt + RViz
- CollisionObjects – static collision objects for the planning scene
- ArmMonitoring – arm position viewer
- RobotCameraTF – static transforms between robot base and camera
- Camera – RealSense camera node
- ObstacleMonitor – dynamic obstacle monitor (if used)
- LeafDetection – leaf detection server + visualisation
- ArduinoServer – Arduino communication node for sprayer/vacuum

Wait until all terminals open up and RViz2 pops up. It should have a pre-loaded MoveIt2 scene with the robot, collision planes, detected leaf markers, the attached end-effector and the camera. This is the fully loaded robot scene, ready to take movement commands with real-time obstacle detection and avoidance. 
Add the camera feed topic can be added to Rviz for better visualisation of the table(shows type and detected leaves). 



### Simulation
aaaaaaaa

[
- clear instructions for launching and running the complete system
- example commands (e.g. ros2 launch project-name bringup.launch.py)
- expected behaviour and example outputs
- optional troubleshooting notes
- system should be launched by a single command (i.e. via a launch file, shell script or Docker image), without manual sequencing

]

## Results and Demonstration
- describe how system performs against its design goals
- include quantitative results where possible (i.e. accuracy, repeatability)
- provide photos, figures, videos showing the system in operation
- highlight robustness, adaptability, and innovation
## Discussion and Future Work
- briefly discuss major engineering challenges faced and how they were addressed
- outline opportunities for improvement or extensions (what would you do better for Verion 2.0)
  * having a RGB-D camera on the end effector to better detect leaves with diseases
- summarise what makes your approach novel, creative or particularly effective
## Contributors and Roles
- briefly list team members and describe their primary areas of responsibility (i.e. vision, planning, hardware)
- Hao Yu: vision + obstacle avoidance path planning
- Darshan Komala Sreeramu: path planning + obstacle avoidance path planning
- Daniel Bui: hardware + designing end effector
## Repository Structure
- a short section outlining folder structure of repository
- explain briefly what each main directory contains

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



[
- credit any external libraries, tutorials, or prior codebases
- acknowledge external assistance (demonstrators, other groups)

]