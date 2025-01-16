#!/usr/bin/env python

from ament_index_python.packages import get_package_share_directory
import yaml, os

def read_teleop_configuration():
    teleoperation_config = os.path.join(
        get_package_share_directory('ros2_3d_interface'),
        'config',
        'teleoperation_config.yaml'
    )

    with open(teleoperation_config, 'r') as file:
        teleoperation_config = yaml.safe_load(file)

    return teleoperation_config

def read_3d_configuration():
    config_file = os.path.join(
        get_package_share_directory('ros2_3d_interface'),
        'config',
        '3d_visualizer_config.yaml'
    )

    with open(config_file, 'r') as file:
        config_file = yaml.safe_load(file)

    return config_file

def read_general_configuration():
    general_config = os.path.join(
        get_package_share_directory('ros2_3d_interface'),
        'config',
        'general_config.yaml'
    )

    with open(general_config, 'r') as file:
        general_config = yaml.safe_load(file)

    return general_config

def read_ros2_node_list():
    ros2_node_list = os.path.join(
        get_package_share_directory('ros2_3d_interface'),
        'config',
        'ros2_node_list.yaml'
    )

    with open(ros2_node_list, 'r') as file:
        ros2_node_list = yaml.safe_load(file)

    return ros2_node_list

def read_ros2_topic_list():
    ros2_topic_list = os.path.join(
        get_package_share_directory('ros2_3d_interface'),
        'config',
        'ros2_topic_list.yaml'
    )

    with open(ros2_topic_list, 'r') as file:
        ros2_topic_list = yaml.safe_load(file)

    return ros2_topic_list

def read_robot_information():
    robot_information = os.path.join(
        get_package_share_directory('ros2_3d_interface'),
        'config',
        'robot_information.yaml'
    )

    with open(robot_information, 'r') as file:
        robot_information = yaml.safe_load(file)

    return robot_information