# 3d_teleoperation_interface

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)


| Verify Line Of Sight                                  | 3D Visualizer of the trajectory                     |
|-------------------------------------------------------|-----------------------------------------------------|
| <img src="https://github.com/hucebot/3d_teleoperation_interface/blob/dev/images/line_of_sight.gif" alt="Line of Sight Verification GIF" width="400"> | <img src="https://github.com/hucebot/3d_teleoperation_interface/blob/dev/images/trajectory_visualization.gif" alt="Trajectory Visualization GIF" width="400"> |


## Table of Contents

## Repository Overview

# Get Started

## Prerequisites

The following dependencies are required to use this package:
- Docker
- NVIDIA Container Toolkit (if you are using an NVIDIA GPU)

## Installation

### From Docker
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command:

```bash
sh build_development.sh
```

To run the development container, use the following command:

```bash
sh run_dev_docker.sh
```

# Usage

## Without ROS2

There is a visualizer tool to visualize the point cloud data in the folder `viewer`, and it's specting a path for a `.npz` file with the point cloud data.

```bash
python3 viewer/viewer.py --path <path_to_npz_file>
```

## With ROS2

### Save point cloud data
There is a ROS2 node that saves the point cloud data to a `.npz` file. To run the node, use the following command:

```bash
ros2 run ros2_3d_interface 3d_recorder
```

### Visualizer without RViz - PC Recorded
There is a ROS2 node that visualizes the point cloud data from a `.npz` file. To run the node, use the following command:

```bash
ros2 run ros2_3d_interface 3d_interface --path <path_to_npz_file>
```

### Visualizer without rviz - PC Real Time
There is a ros2 node that visualizes the point cloud data in real time without using rviz. To run the node, use the following command:

```bash
ros2 run ros2_3d_interface 3d_real_time
```

### Publish trajectory - Fixed data - Testing
There is a ros2 node that publish the trajectory that we should follow. To run the node, use the following command:

```bash
ros2 run ros2_3d_interface publish_trajectory
```

### Publish point cloud data
To publish the point cloud data, it will depent on the camera you are using. For the Intel RealSense D435i, you can use the following command:

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true enable_rgbd:=true align_depth.enable:=true enable_sync:=true enable_depth:=true enable_color:=true
```

### Teleoperation
There is a ROS2 node that allows you visualize all the information of the robot, this means cameras, robot information, robot state, point cloud, trajectories, etc. To run the node, use the following command:
```bash
ros2 run ros2_3d_interface teleoperation
```