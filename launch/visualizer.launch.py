from launch import LaunchDescription

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('ros2_3d_interface'),
        'config',
        'general_configuration.yaml'
    )

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    return LaunchDescription(
        generate_nodes(config)
    )

def generate_nodes(config_file):
    # General parameters
    render_pyramid = config_file['3d_viewer']['general']['render_pyramid']
    render_trajectory = config_file['3d_viewer']['general']['render_trajectory']
    render_robot = config_file['3d_viewer']['general']['render_robot']
    render_image = config_file['3d_viewer']['general']['render_image']
    render_hz = config_file['3d_viewer']['general']['render_hz']
    camera_velocity = config_file['3d_viewer']['general']['camera_velocity']

    robot_model = config_file['3d_viewer']['robot']['model']
    robot_version = config_file['3d_viewer']['robot']['version']

    rgb_image_width = config_file['3d_viewer']['rgb_image']['width']
    rgb_image_height = config_file['3d_viewer']['rgb_image']['height']
    rgb_image_topic = config_file['3d_viewer']['rgb_image']['topic']
    visualizer_x = config_file['3d_viewer']['rgb_image']['visualizer_x']
    visualizer_y = config_file['3d_viewer']['rgb_image']['visualizer_y']
    visualizer_z = config_file['3d_viewer']['rgb_image']['visualizer_z']
    visualizer_width = config_file['3d_viewer']['rgb_image']['visualizer_width']
    visualizer_height = config_file['3d_viewer']['rgb_image']['visualizer_height']

    use_streamdeck = config_file['3d_viewer']['streamdeck']['use_streamdeck']

    reset_view_topic = config_file['3d_viewer']['streamdeck']['reset_view_topic']

    point_cloud_width = config_file['3d_viewer']['point_cloud']['width']
    point_cloud_height = config_file['3d_viewer']['point_cloud']['height']
    point_cloud_size_multiplier = config_file['3d_viewer']['point_cloud']['size_multiplier']
    point_size = config_file['3d_viewer']['point_cloud']['point_size']
    hfov = config_file['3d_viewer']['point_cloud']['hfov']
    vfov = config_file['3d_viewer']['point_cloud']['vfov']
    point_cloud_topic = config_file['3d_viewer']['point_cloud']['topic']

    # Trajectory parameters
    trajectory_points_topic = config_file['trajectory']['trajectory_points_topic']
    dummy_trajectory = config_file['trajectory']['dummy_trajectory']

    node_list = []

    # This could be main, upside_down or side
    for views in ['main']:
        node = Node(
            package='ros2_3d_interface',
            executable='3d_viewer.py',
            name=f'{views}_camera_window',
            output='screen',
            parameters=[
                {
                    'window_name': config_file['3d_viewer'][f'{views}_window']['name'],
                    'window_width': config_file['3d_viewer'][f'{views}_window']['width'],
                    'window_height': config_file['3d_viewer'][f'{views}_window']['height'],
                    'camera_x': config_file['3d_viewer'][f'{views}_window']['camera_x'],
                    'camera_y': config_file['3d_viewer'][f'{views}_window']['camera_y'],
                    'camera_z': config_file['3d_viewer'][f'{views}_window']['camera_z'],
                    'camera_yaw': config_file['3d_viewer'][f'{views}_window']['camera_yaw'],
                    'camera_pitch': config_file['3d_viewer'][f'{views}_window']['camera_pitch'],
                    'camera_roll': config_file['3d_viewer'][f'{views}_window']['camera_roll'],
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
                    'render_pyramid': render_pyramid,
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