# Leaf Sorting and Sprayer Arm

This repository contains the ROS 2 Humble workspace for a UR5e-based leaf sorting and sprayer arm.
The system uses a pole-mounted Intel RealSense camera and a custom sprayer/vacuum end-effector to detect leaves in 3D, plan collision-free trajectories with MoveIt, and pick-and-place healthy leaves into a box or actuate a pump via an Arduino bridge to spray pesticide on diseased leaves – all in a closed-loop pipeline.

The project was developed and tested on **Ubuntu 22.04 + ROS 2 Humble** with a **UR5e**, **RealSense RGB-D camera**, and a **custom built pesticide sprayer/vacuum end-effector**.

> **Demo video:** _[link coming soon]_  
> (Short clip of the full cycle: detect leaf → move → spray/vacuum → return home.)

---

## Table of Contents

- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
  - [Node and Communication Graph](#node-and-communication-graph)
  - [Package-Level Architecture](#package-level-architecture)
  - [Task Logic and State Machine](#task-logic-and-state-machine)
- [Technical Components](#technical-components)
  - [Computer Vision Pipeline](#computer-vision-pipeline)
  - [Robot and Custom End-Effector](#robot-and-custom-end-effector)
  - [System Visualisation](#system-visualisation)
  - [Closed-Loop Operation](#closed-loop-operation)
- [Installation and Setup](#installation-and-setup)
  - [Software Dependencies](#software-dependencies)
  - [Workspace Setup](#workspace-setup)
  - [Hardware Setup](#hardware-setup)
  - [Calibration and Configuration](#calibration-and-configuration)
- [Running the System](#running-the-system)
  - [Real Robot Bringup](#real-robot-bringup)
  - [Automation Task Launch](#automation-task-launch)
  - [Expected Behaviour](#expected-behaviour)
  - [Troubleshooting](#troubleshooting)
- [Results and Demonstration](#results-and-demonstration)
  - [Quantitative Performance](#quantitative-performance)
  - [Qualitative Observations](#qualitative-observations)
  - [Videos and Media](#videos-and-media)
- [Discussion and Future Work](#discussion-and-future-work)
- [Contributors and Roles](#contributors-and-roles)
- [Repository Structure](#repository-structure)
- [References and Acknowledgements](#references-and-acknowledgements)
