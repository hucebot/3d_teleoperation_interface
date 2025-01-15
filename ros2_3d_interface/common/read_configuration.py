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