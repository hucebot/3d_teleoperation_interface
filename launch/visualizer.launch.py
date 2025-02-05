from launch import LaunchDescription

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

from ros2_3d_interface.common.read_configuration import read_3d_configuration, read_general_configuration

def generate_launch_description():
    config_3d_file = read_3d_configuration()
    general_config_file = read_general_configuration()

    return LaunchDescription(
        generate_nodes(config_3d_file, general_config_file)
    )

def generate_nodes(config_3d_file, general_config_file):
    # General parameters
    use_streamdeck = general_config_file['general']['use_streamdeck']
    reset_view_topic = config_3d_file['streamdeck']['reset_view_topic']
    use_robot_window = general_config_file['general']['use_robot_window']
    use_network_window = general_config_file['general']['use_network_window']
    use_stream_window = general_config_file['general']['use_stream_window']

    # Trajectory parameters
    trajectory_points_topic = config_3d_file['trajectory']['topic']
    use_dummy_trajectory = general_config_file['general']['use_dummy_trajectory']

    node_list = []

    node = Node(
        package='ros2_3d_interface',
        executable='3d_viewer.py',
        name='main_window',
        output='screen',
        parameters=[
            {
            }
        ]
    )

    node_list.append(node)

    if use_streamdeck:
        streamdeck_node = Node(
            package='ros2_3d_interface',
            executable='streamdeck.py',
            name='streamdeck',
            output='screen',
            parameters=[
                {
                    'reset_view_topic': reset_view_topic
                }
            ]
        )

        node_list.append(streamdeck_node)

    if use_robot_window:
        robot_window_node = Node(
            package='ros2_3d_interface',
            executable='robot_window.py',
            name='robot_window',
            output='screen',
            parameters=[
                {
                }
            ]
        )

        node_list.append(robot_window_node)

    if use_network_window:
        network_window_node = Node(
            package='ros2_3d_interface',
            executable='network_window.py',
            name='network_window',
            output='screen',
            parameters=[
                {
                }
            ]
        )

        node_list.append(network_window_node)

    if use_stream_window:
        stream_window_node = Node(
            package='ros2_3d_interface',
            executable='stream_window.py',
            name='stream_window',
            output='screen',
            parameters=[
                {
                }
            ]
        )

        node_list.append(stream_window_node)

    if use_dummy_trajectory:
        dummy_trajectory_node = Node(
            package='ros2_3d_interface',
            executable='dummy_trajectory.py',
            name='dummy_trajectory',
            output='screen',
            parameters=[
                {
                    'trajectory_points_topic': trajectory_points_topic,
                }
            ]
        )

        node_list.append(dummy_trajectory_node)

    return node_list