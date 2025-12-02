# pesticide-sprayer-arm
UR5e arm with custom end-effector that sprays pesticide on diseased leaves

# Table of Contents
# Project Overview (Hayden)
- short description of task/problem your system solves, including intended "customer" or end-user
- summary of robot's functionality
- short video (10-30s) of the robot completing one full cycle/operation demonstrating closed-loop behaviour and visualisation (embedded or external link to Youtube/OneDrive/Google Drive video)
# System Architecture (Hayden)
- diagram of ROS2 nodes, topics, services and actions (from rqt_graph or custom schematic)
- package-level architecture diagram showing node interactions and topics
- behvaiour tree or state-machine diagram showing closed loop system behaviour
- brief description of the function of each node
- any custom message types or interfaces should be listed and explained. 
# Technical Components (Hayden + Daniel)
## Computer Vision
describe vision pipeline and how it contributes to the task.
## Custom End-Effector
provide photos/renders, assembly details, engineering drawings, control overview and integration details.
Engineering Drawings:
* [whole assembly stl file](CustomEndEffector/STL%20Files/FullAssembly.stl)
* [whole assembly drawing](CustomEndEffector/Drawings/FullAssemblyDrawing.pdf)
* [closing mount stl file](CustomEndEffector/STL%20Files/ClosingMount.stl)
* [closing mount drawing](CustomEndEffector/Drawings/ClosingMount.pdf)
* [vacuumPumpMount stl file](CustomEndEffector/STL%20Files/VacuumPumpMount.stl)
* [vacuumPumpMount drawing](CustomEndEffector/Drawings/VacuumPumpMount.pdf)
* [SprayMount stl file](CustomEndEffector/STL%20Files/SprayMountBox.stl)
* [SprayMount drawing](CustomEndEffector/Drawings/SprayMountBox.pdf)
* [providedMount stl file](CustomEndEffector/STL%20Files/ProvidedMount.stl)
* [providedMount drawing](CustomEndEffector/Drawings/ProvidedMount.pdf)
* [endEffectorMount stl file](CustomEndEffector/STL%20Files/EndEffectorComponent.stl)
* [endEffectorMount drawing](CustomEndEffector/Drawings/EndEffectorComponentDrawing.pdf)
## System Visualisation
explain how system is visualised (RViz) and what it demonstrates
## Closed-Loop Operation
describe the feedback method and how it adapts system behaviour in real time

# Installation and Setup (Hayden + Daniel)
- step-by-step installation instructions for dependencies and workspace setup
- hardware setup information (UR5e connection, camera, Teensy, etc.)
  * The UR5e robot must be running the ROS program and initially in the home position
  * This project uses the provided fixed depth camera on the table. 
  * An Arduino UNO board is used to control the vacuum and spray motors. 
  * communication is done over UART and ROS2 Client/Server communication between the robot and the Arduino board
- any environment variables, configuration files, or calibration prcedures required to run the system (can assume there is some sort of hand-eye calibration already present in the system)
  * having yellow crosses on the leafs provide a clear indication of bad leaves
# Running the System
- clear instructions for launching and running the complete system
- example commands (e.g. ros2 launch project-name bringup.launch.py)
- expected behaviour and example outputs
- optional troubleshooting notes
- system should be launched by a single command (i.e. via a launch file, shell script or Docker image), without manual sequencing
# Results and Demonstration
- describe how system performs against its design goals
- include quantitative results where possible (i.e. accuracy, repeatability)
- provide photos, figures, videos showing the system in operation
- highlight robustness, adaptability, and innovation
# Discussion and Future Work
- briefly discuss major engineering challenges faced and how they were addressed
  * Reliable Leaf Detection
     * YOLO peformed well, but the change in environmnet introduced noise. This was resolved by adjusting the HSV values to stabilise detections
  * Stable robot-camera calibration
     * reconnectig the camera could shift extrinsics, causing markers to appear in the wrong place in RViz2.
  * End Effector airflow and spray consistency
     * Achieving precise spryaing without affecting nearby leaves required tuning the z parameter
- outline opportunities for improvement or extensions (what would you do better for Version 2.0)
  * having a RGB-D camera on the end effector would significantly improve disease detection and 3D localisation from close range
  * Better disease classification: integrate a more detailed plant model to classify leaf health more accurately (yellow spots, fungal infection)
  * Full Robot Workspace Mapping: using a structured-light scanning to model the whole plant geometry for improved motion planning
- summarise what makes your approach novel, creative or particularly effective
  * Fully closed-loop operation: system dynamically adapts its motion to real-time leaf detections rather than relying on hard-coded positions
  * Dual function end-effector: single tool head performs both leaf removal and precise spraying, reducing hardware complexity
  * Low hardware cost wiht high flexibility:
  * Modular Design: the motor mounts can be easily replaced and the closing mount makes it easy to diagnose issues with wiring
# Contributors and Roles
|Team Member            |Primary Responsibilities|
|-----------------------|------------------------|
|Hao Yu                 | Led computer vision development (YOLO detection and depth processing) <br> Designed and implemented obstacle avoidance path planning <br> Integrated all software and hardware components into final working system|
|Darshan Komala Sreeramu| Developed robust path planning algorithms <br> Added saftey planes and robot joint constraints to ensure safe robot motion <br> Contributed to obstacle-avoidance planning and motion-control refinement|
|Daniel Bui             | Responsible for hardware assembly and electronics <br> Designed and 3D printed the custom end-effector for leaf pick-up and spraying|

# Repository Structure
- a short section outlining folder structure of repository
- explain briefly what each main directory contains
# References and Acknowledgements
- credit any external libraries, tutorials, or prior codebases
- acknowledge external assistance (demonstrators, other groups)

