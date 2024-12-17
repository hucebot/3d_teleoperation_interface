import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.trajectory_publisher = self.create_publisher(
            MarkerArray,
            '/trajectory_points',
            10)

        self.points = [
            [1.0, 0.5, 0.5],
            [2.0, 1.0, 1.0],
            [3.0, 1.5, 1.0],
            [4.0, 2.0, 1.0],
            [5.0, 2.5, 1.0],
            [6.0, 2.0, 1.0],
            [7.0, 1.5, 1.0],
            [8.0, 0.0, 1.0],
            [8.0, -1.0, 1.0]
        ]

    def publish_trajectory(self):
        marker_array = MarkerArray()
        origin = np.array([0.0, 0.0, 0.0])

        for i, point in enumerate(self.points):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0] / 10
            marker.pose.position.y = point[1] / 10
            marker.pose.position.z = point[2] / 10
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.trajectory_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        while rclpy.ok():
            node.publish_trajectory()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
