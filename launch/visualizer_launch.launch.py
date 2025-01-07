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
    # 3D Viewer parameters
    window_name = config_file['3d_viewer']['window_name']
    window_width = config_file['3d_viewer']['window_width']
    window_height = config_file['3d_viewer']['window_height']
    point_cloud_width = config_file['3d_viewer']['point_cloud_width']
    point_cloud_height = config_file['3d_viewer']['point_cloud_height']
    point_cloud_size_multiplier = config_file['3d_viewer']['point_cloud_size_multiplier']
    render_pyramid = config_file['3d_viewer']['render_pyramid']
    render_trajectory = config_file['3d_viewer']['render_trajectory']
    fov = config_file['3d_viewer']['fov']
    render_hz = config_file['3d_viewer']['render_hz']
    camera_velocity = config_file['3d_viewer']['camera_velocity']
    point_cloud_topic = config_file['3d_viewer']['point_cloud_topic']

    # Trajectory parameters
    trajectory_points_topic = config_file['trajectory']['trajectory_points_topic']
    dummy_trajectory = config_file['trajectory']['dummy_trajectory']

    visualizer_node = Node(
        package='ros2_3d_interface',
        executable='3d_viewer.py',
        name='viewer',
        output='screen',
        parameters=[
            {
                'window_name': window_name,
                'window_width': window_width,
                'window_height': window_height,
                'point_cloud_width': point_cloud_width,
                'point_cloud_height': point_cloud_height,
                'point_cloud_size_multiplier': point_cloud_size_multiplier,
                'render_pyramid': render_pyramid,
                'render_trajectory': render_trajectory,
                'fov': fov,
                'render_hz': render_hz,
                'camera_velocity': camera_velocity,
                'point_cloud_topic': point_cloud_topic,
                'trajectory_points_topic': trajectory_points_topic
            }
        ]
    )

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

        return [visualizer_node, dummy_trajectory_node]

    else:
        return [visualizer_node]