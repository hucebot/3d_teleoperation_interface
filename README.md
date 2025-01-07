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
    - [Publish point cloud data](#publish-point-cloud-data)
    - [3D Visualizer](#3d-visualizer)
        - [Configuration File](#configuration-file)

## Repository Overview
```plaintext
.
├── CMakeLists.txt
├── config
│   ├── d435i_calibration.yaml
│   ├── general_configuration.yaml
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
│   └── visualizer_launch.launch.py
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
│   ├── 3d_viewer.py
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

## Publish point cloud data
To publish the point cloud data, it will depent on the camera you are using. For the Orbbec camera (Femto Bolt), use the following command:

```bash
ros2 launch orbbec_camera femto_bolt.launch.py -enable_point_cloud:=true enable_colored_point_cloud:=true
```

## 3D Visualizer
There is launch file that will start the visualizer tool to visualize the point cloud data and the trajectory. To start the visualizer, use the following command:

```bash
ros2 launch  ros2_3d_interface visualizer.launch.py
```

### Configuration File
The configuration located in the `config` folder is used to setup everything for the visualizer. The file `general_configuration.yaml` has the following structure:

```yaml
3d_viewer:
  window_name: Name that will be displayed in the window
  window_width: Window width for the visualizer
  window_height: Window height for the visualizer
  point_cloud_width: Point cloud width
  point_cloud_height: Point cloud height
  point_cloud_size_multiplier: Multiplier for the point cloud size
  render_pyramid: Boolean to render the pyramid view
  render_trajectory: Boolean to render the trajectory
  fov: Field of view for the camera
  render_hz: Render frequency
  camera_velocity: Camera velocity
  point_cloud_topic: Point cloud topic used to visualize the point cloud data
  
trajectory:
  trajectory_points_topic: Trajectory points topic
  dummy_trajectory: Boolean to use a dummy trajectory
```
