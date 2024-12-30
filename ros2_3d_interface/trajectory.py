import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.trajectory = [
            [1.1, 0.5, 0.1],
            [1.1, -0.3, 0.2],
            [1.1, 0.1, 0.3],
        ]

        self.trajectory_publisher = self.create_publisher(
            MarkerArray,
            '/trajectory_points',
            10
        )


    def publish_trajectory(self):
        marker_array = MarkerArray()
        for i, point in enumerate(self.trajectory):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
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
            rclpy.spin_once(node, timeout_sec=0.1)
            node.publish_trajectory()
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
