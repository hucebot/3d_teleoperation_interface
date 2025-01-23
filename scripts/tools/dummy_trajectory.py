#!/usr/bin/env python3

import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension, Int16
from ros2_3d_interface.common.read_configuration import read_3d_configuration

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.get_logger().info('Iniciando TrajectoryNode en Python...')

        self.config_file = read_3d_configuration()

        self.total_trajectories = self.config_file['trajectory']['total_trajectories']
        self.update_period = self.config_file['trajectory']['update_rate']
        self.number_of_points = self.config_file['trajectory']['number_of_points_per_trajectory']
        self.topic = self.config_file['trajectory']['topic']

        self.base_trajectory = [
            (0.0, 0.5, 0.0),
            (0.5, 1.3, 0.2),
            (1.0, 2.2, 0.5),
            (2.0, 2.5, 0.9),
            (3.0, 2.8, 1.3),
            (4.0, 3.0, 1.8),
            (5.0, 2.5, 2.0),
            (5.5, 2.0, 2.0),
            (6.0, 1.0, 1.7),
            (6.5, 0.0, 1.2),
            (7.0, -1.0, 1.1),
            (7.5, -2.0, 1.0),
            (8.0, -2.5, 0.9),
            (9.0, -2.5, 0.5),
            (10.0, -2.0, 0.2),
            (10.5, -1.0, 0.1),
        ]

        self.base_trajectory = [(x / 10, y / 10, z / 10) for (x, y, z) in self.base_trajectory]

        self.current_step = 1

        self.trajectories = self.generate_trajectories(self.total_trajectories)

        self.trajectories_publisher = self.create_publisher(Float64MultiArray, self.topic, 10)

        self.timer = self.create_timer(self.update_period, self.publish_incremental_trajectories)

    def generate_trajectories(self, num_trajectories: int):
        trajectories = []
        for _ in range(num_trajectories):
            random_traj = []

            for (bx, by, bz) in self.base_trajectory:
                dx = random.uniform(-1.0, 1.0)
                dy = random.uniform(-1.0, 1.0)
                dz = random.uniform(-1.0, 1.0)

                length = math.sqrt(dx * dx + dy * dy + dz * dz)
                if length > 1e-9:
                    dx /= length
                    dy /= length
                    dz /= length

                r = random.uniform(0.0, 0.05)

                nx = bx + dx * r
                ny = by + dy * r
                nz = bz + dz * r

                random_traj.append((nx, ny, nz))

            trajectories.append(random_traj)

        return trajectories

    def publish_incremental_trajectories(self):
        data = []
        for trajectory in self.trajectories:
            data.extend(trajectory[:self.current_step])

        multi_array = Float64MultiArray()
        multi_array.layout = MultiArrayLayout()
        multi_array.layout.dim = [
            MultiArrayDimension(label='trajectories', size=self.total_trajectories, stride=len(data)),
            MultiArrayDimension(label='points', size=min(self.current_step, self.number_of_points), stride=3),
            MultiArrayDimension(label='xyz', size=3, stride=3),
        ]
        multi_array.data = [coord for point in data for coord in point]

        self.trajectories_publisher.publish(multi_array)

        self.current_step += 1
        if self.current_step > self.number_of_points:
            self.current_step = 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
