# OAAC of MSE for GR via OTO

## Overview

```sh
├── auto_nav/  # Bringup launch and PID Tracker
├── bspline_optimization/ # Cpp implementation of online bspline optimization
├── CMakeLists.txt
├── laser_scan_matcher/ # Scan-to-Scan point-to-line lidar registration
├── offline_trajectory/ # Trajectory for comparison
├── turtlebot3_bringup/ # Turtlebot bringup files
├── turtlebot3_description/ # Turtlebot model
├── turtlebot3_simulations/ # Turtlebot gazebo
└── xf_mic_online/ # Online sound DOF estimator
```

## Usage

The code is tested on ubuntu 20.04 ROS1 noetic ARM64 platform only. To be specific, *Jetson orin NX* and *Jetson Xavier NX*.

Before using the code, you have to install [NLOPT](https://github.com/stevengj/nlopt) first. Then use vanilla `catkin_make` to compile.

Use `roslaunch auto_nav ref_traj_icp_slam_pid_real.launch` to boot the whole robot stack including Turtlebot driver, lidar driver ,pid follower and EKF extrinsic calibrator.

Run `bspline_optimization_node` to listen to extrinsic data and publish results, TF data and generate optimized B-spline curve online.

Use `roslaunch xf_mic_online mic_init.launch` to start DOA estimation.

Boot microphone array.

## Code References
Apart from references in the paper, we have borrowed from:
1. [MPC ROS](https://github.com/Geonhee-LEE/mpc_ros) *MPC trajectory generator and tracker.* Was using mpc to track but always fall into local minima, borrowed offline trajectory generation script only.
2. [Auto Nav](https://github.com/SCAU-RM-NAV/rm2023_auto_sentry_ws/tree/main/src/auto_nav) *PID Tracker*. Adapted to turtlebot base.
3. [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)*Turtlebot ros*
4. [Scan Match](https://github.com/CCNYRoboticsLab/scan_tools) *Simple scan to scan implementation utilizing point to line metric.*
5. [NLOPT](https://github.com/stevengj/nlopt) *Nonlinear Optimizer*