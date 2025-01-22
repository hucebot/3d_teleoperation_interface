#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math
import random

from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.get_logger().info('Iniciando TrajectoryNode en Python...')

        self.total_trajectories = 40
        self.number_of_points = 16

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


        self.publisher_ = self.create_publisher(Float64MultiArray, '/trajectory_points', 10)

        period = 0.5

        self.timer_ = self.create_timer(period, self.publish_trajectories)

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

            for point in random_traj:
                trajectories.extend(point)

        return trajectories

    def publish_trajectories(self):
        data = self.generate_trajectories(self.total_trajectories)

        multi_array = Float64MultiArray()
        multi_array.layout = MultiArrayLayout()
        multi_array.layout.dim = [
            MultiArrayDimension(label='trajectories', size=self.total_trajectories, stride=len(data)),
            MultiArrayDimension(label='points', size=self.number_of_points, stride=self.number_of_points * 3),
            MultiArrayDimension(label='xyz', size=3, stride=3),
        ]
        multi_array.data = data

        self.publisher_.publish(multi_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
