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

# Installation and Setup (Hayden + Daniel)
- step-by-step installation instructions for dependencies and workspace setup
- hardware setup information (UR5e connection, camera, Teensy, etc.)
  * camera is mounted on the provided bracket
  * Arduino is used to code the motor configuration
  * communication is done over UART and ROS2 Client/Server communication
- any environment variables, configuration files, or calibration prcedures required to run the system (can assume there is some sort of hand-eye calibration already present in the system)
  * having yellow crosses on the leafs provide a clear indication of bad/good leaves
# Running the System (Darshan)
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
- outline opportunities for improvement or extensions (what would you do better for Verion 2.0)
  * having a RGB-D camera on the end effector to better detect leaves with diseases
- summarise what makes your approach novel, creative or particularly effective
# Contributors and Roles
- briefly list team members and describe their primary areas of responsibility (i.e. vision, planning, hardware)
- Hao Yu: vision + obstacle avoidance path planning
- Darshan Komala Sreeramu: path planning + obstacle avoidance path planning
- Daniel Bui: hardware + designing end effector
# Repository Structure
- a short section outlining folder structure of repository
- explain briefly what each main directory contains
# References and Acknowledgements
- credit any external libraries, tutorials, or prior codebases
- acknowledge external assistance (demonstrators, other groups)

