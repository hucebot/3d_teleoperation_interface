# 3d_teleoperation_interface

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)


| Verify Line Of Sight                                  | 3D Visualizer of the trajectory                     |
|-------------------------------------------------------|-----------------------------------------------------|
| <img src="https://github.com/hucebot/3d_teleoperation_interface/blob/dev/images/line_of_sight.gif" alt="Line of Sight Verification GIF" width="400"> | <img src="https://github.com/hucebot/3d_teleoperation_interface/blob/dev/images/trajectory_visualization.gif" alt="Trajectory Visualization GIF" width="400"> |


## Table of Contents
- [Repository Overview](#repository-overview)
- [Get Started](#get-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
    - [From Docker](#from-docker)
- [Usage](#usage)
    - [Without ROS2](#without-ros2)
    - [With ROS2](#with-ros2)
        - [Save point cloud data](#save-point-cloud-data)
        - [Visualizer without RViz - PC Recorded](#visualizer-without-rviz---pc-recorded)
        - [Visualizer without rviz - PC Real Time](#visualizer-without-rviz---pc-real-time)
        - [Publish trajectory - Fixed data - Testing](#publish-trajectory---fixed-data---testing)
        - [Publish point cloud data](#publish-point-cloud-data)
        - [Teleoperation](#teleoperation)

## Repository Overview
```plaintext
.
├── CMakeLists.txt
├── config
│   ├── d435i_calibration.yaml
│   └── rviz_config.rviz
├── docker
│   ├── build_development.sh
│   ├── deploy.Dockerfile
│   ├── devel.Dockerfile
│   └── run_dev_docker.sh
├── images
│   ├── line_of_sight.gif
│   └── trajectory_visualization.gif
├── launch
├── LINCESE
├── models
│   └── gripper
│       ├── gripper.20241210-203224.FCBak
│       ├── gripper.FCStd
│       └── gripper.obj
├── package.xml
├── README.md
├── resource
│   └── ros2_3d_interface
├── ros2_3d_interface
│   ├── __init__.py
│   └── utilities
│       ├── camera.py
│       ├── __init__.py
│       ├── __pycache__
│       │   └── LineGeometry.cpython-310.pyc
│       ├── shape.py
│       ├── utils.py
│       └── viewer.py
├── scripts
│   ├── 3d_interface.py
│   ├── 3d_real_time.py
│   ├── __init__.py
│   ├── record_pcl.py
│   ├── streamdeck.py
│   ├── trajectory_bridge.py
│   └── trajectory.py
├── setup.cfg
├── src
│   ├── cloud_separation.cpp
│   ├── point_cloud_publisher.cpp
│   ├── trajectory.cpp
│   └── verify_line_of_sight.cpp
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── viewer
    ├── 3d_viewer.py
    └── viewer_opengl
        ├── camera.py
        ├── __init__.py
        ├── shape.py
        ├── utils.py
        └── viewer.py
```

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
To publish the point cloud data, it will depent on the camera you are using. For the Orbbec Astra camera (Femto Bolt), use the following command:

```bash
ros2 launch orbbec_camera femto_bolt.launch.py -enable_point_cloud:=true enable_colored_point_cloud:=true
```

### Teleoperation
There is a ROS2 node that allows you visualize all the information of the robot, this means cameras, robot information, robot state, point cloud, trajectories, etc. To run the node, use the following command:
```bash
ros2 run ros2_3d_interface teleoperation
```