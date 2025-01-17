from launch import LaunchDescription

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

from ros2_3d_interface.common.read_configuration import read_general_configuration, read_3d_configuration

def generate_launch_description():
    config_3d_file = read_3d_configuration()

    general_config = read_general_configuration()

    return LaunchDescription(
        generate_nodes(config_3d_file, general_config)
    )

def generate_nodes(config_3d_file, general_config):
    # General parameters
    render_pyramid = general_config['general']['render_pyramid']
    render_trajectory = general_config['general']['render_trajectory']
    render_robot = general_config['general']['render_robot']
    render_image = general_config['general']['render_image']
    render_hz = general_config['general']['render_hz']
    camera_velocity = general_config['general']['camera_velocity']

    robot_model = config_3d_file['robot']['model']
    robot_version = config_3d_file['robot']['version']

    rgb_image_width = config_3d_file['rgb_image']['width']
    rgb_image_height = config_3d_file['rgb_image']['height']
    rgb_image_topic = config_3d_file['rgb_image']['topic']
    visualizer_x = config_3d_file['rgb_image']['visualizer_x']
    visualizer_y = config_3d_file['rgb_image']['visualizer_y']
    visualizer_z = config_3d_file['rgb_image']['visualizer_z']
    visualizer_width = config_3d_file['rgb_image']['visualizer_width']
    visualizer_height = config_3d_file['rgb_image']['visualizer_height']

    use_streamdeck = config_3d_file['streamdeck']['use_streamdeck']

    reset_view_topic = config_3d_file['streamdeck']['reset_view_topic']

    point_cloud_width = config_3d_file['point_cloud']['width']
    point_cloud_height = config_3d_file['point_cloud']['height']
    point_cloud_size_multiplier = config_3d_file['point_cloud']['size_multiplier']
    point_size = config_3d_file['point_cloud']['point_size']
    hfov = config_3d_file['point_cloud']['hfov']
    vfov = config_3d_file['point_cloud']['vfov']
    point_cloud_topic = config_3d_file['point_cloud']['topic']

    # Trajectory parameters
    trajectory_points_topic = config_3d_file['trajectory']['trajectory_points_topic']
    dummy_trajectory = config_3d_file['trajectory']['dummy_trajectory']

    name = config_3d_file['main_window']['name']
    window_width = config_3d_file['main_window']['width']
    window_height = config_3d_file['main_window']['height']

    frontal_camera_x = config_3d_file['main_window']['frontal_camera_x']
    frontal_camera_y = config_3d_file['main_window']['frontal_camera_y']
    frontal_camera_z = config_3d_file['main_window']['frontal_camera_z']
    frontal_camera_yaw = config_3d_file['main_window']['frontal_camera_yaw']
    frontal_camera_pitch = config_3d_file['main_window']['frontal_camera_pitch']
    frontal_camera_roll = config_3d_file['main_window']['frontal_camera_roll']

    upper_camera_x = config_3d_file['main_window']['upper_camera_x']
    upper_camera_y = config_3d_file['main_window']['upper_camera_y']
    upper_camera_z = config_3d_file['main_window']['upper_camera_z']
    upper_camera_yaw = config_3d_file['main_window']['upper_camera_yaw']
    upper_camera_pitch = config_3d_file['main_window']['upper_camera_pitch']
    upper_camera_roll = config_3d_file['main_window']['upper_camera_roll']

    side_camera_x = config_3d_file['main_window']['side_camera_x']
    side_camera_y = config_3d_file['main_window']['side_camera_y']
    side_camera_z = config_3d_file['main_window']['side_camera_z']
    side_camera_yaw = config_3d_file['main_window']['side_camera_yaw']
    side_camera_pitch = config_3d_file['main_window']['side_camera_pitch']
    side_camera_roll = config_3d_file['main_window']['side_camera_roll']

    node_list = []

    node = Node(
        package='ros2_3d_interface',
        executable='3d_viewer.py',
        name='main_window',
        output='screen',
        parameters=[
            {
                'render_pyramid': render_pyramid,
                'window_name': name,
                'window_width': window_width,
                'window_height': window_height,
                'frontal_camera_x': frontal_camera_x,
                'frontal_camera_y': frontal_camera_y,
                'frontal_camera_z': frontal_camera_z,
                'frontal_camera_yaw': frontal_camera_yaw,
                'frontal_camera_pitch': frontal_camera_pitch,
                'frontal_camera_roll': frontal_camera_roll,
                'upper_camera_x': upper_camera_x,
                'upper_camera_y': upper_camera_y,
                'upper_camera_z': upper_camera_z,
                'upper_camera_yaw': upper_camera_yaw,
                'upper_camera_pitch': upper_camera_pitch,
                'upper_camera_roll': upper_camera_roll,
                'side_camera_x': side_camera_x,
                'side_camera_y': side_camera_y,
                'side_camera_z': side_camera_z,
                'side_camera_yaw': side_camera_yaw,
                'side_camera_pitch': side_camera_pitch,
                'side_camera_roll': side_camera_roll,
                'rgb_image_width': rgb_image_width,
                'rgb_image_height': rgb_image_height,
                'visualizer_x': visualizer_x,
                'visualizer_y': visualizer_y,
                'visualizer_z': visualizer_z,
                'visualizer_width': visualizer_width,
                'visualizer_height': visualizer_height,
                'point_cloud_width': point_cloud_width,
                'point_cloud_height': point_cloud_height,
                'point_cloud_size_multiplier': point_cloud_size_multiplier,
                'point_size': point_size,
                'render_trajectory': render_trajectory,
                'render_robot': render_robot,
                'robot_model': robot_model,
                'render_image': render_image,
                'reset_view_topic': reset_view_topic,
                'hfov': hfov,
                'vfov': vfov,
                'render_hz': render_hz,
                'camera_velocity': camera_velocity,
                'point_cloud_topic': point_cloud_topic,
                'trajectory_points_topic': trajectory_points_topic,
                'rgb_image_topic': rgb_image_topic
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

    if dummy_trajectory:
        dummy_trajectory_node = Node(
            package='ros2_3d_interface',
            executable='publish_trajectory',
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