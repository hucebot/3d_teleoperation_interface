import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import torch


class TrajectoryVerify(Node):
    def __init__(self):
        super().__init__('trajectory_verify')

        self.origin = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.trajectory_points = []
        self.reduced_point_cloud = None
        self.origin_tensor = torch.tensor(self.origin, dtype=torch.float32, device="cuda")

        self.trajectory_subscription = self.create_subscription(
            MarkerArray,
            '/trajectory_points',
            self.trajectory_cb,
            10
        )

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_cb,
            10
        )

        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.point_cloud_cb,
            10
        )

        self.trajectory_verify_publisher = self.create_publisher(
            MarkerArray,
            '/trajectory_verify',
            10
        )

    def trajectory_cb(self, msg):
        self.trajectory_points = [
            np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z], dtype=np.float32)
            for marker in msg.markers
        ]

        self.trajectory_tensor = torch.tensor(
            np.array(self.trajectory_points),
            dtype=torch.float32,
            device="cuda"
        )

        self.destroy_subscription(self.trajectory_subscription)

    def point_cloud_cb(self, msg):
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.reduced_point_cloud = torch.tensor(cloud_points, dtype=torch.float32, device="cuda")

    def verify_trajectory(self):
        if self.reduced_point_cloud is None or not self.trajectory_points:
            return

        marker_array = MarkerArray()

        for i, point in enumerate(self.trajectory_tensor):
            origin_to_point = point - self.origin_tensor
            direction = origin_to_point / torch.norm(origin_to_point)

            projections = torch.sum((self.reduced_point_cloud - self.origin_tensor) * direction, dim=1)

            within_range = (projections > 0) & (projections < torch.norm(origin_to_point))
            valid_points = self.reduced_point_cloud[within_range]

            if valid_points.size(0) > 0:
                projections = torch.sum((valid_points - self.origin_tensor) * direction, dim=1)
                projected_points = self.origin_tensor + projections[:, None] * direction

                distances = torch.norm(valid_points - projected_points, dim=1)

                obstructed = torch.any(distances < 0.8) 
            else:
                obstructed = False

            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectory_lines"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02

            if obstructed:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 1.0

            start_point = Point(x=float(self.origin[0]), y=float(self.origin[1]), z=float(self.origin[2]))
            end_point = Point(x=float(point[0].cpu()), y=float(point[1].cpu()), z=float(point[2].cpu()))
            marker.points.append(start_point)
            marker.points.append(end_point)

            marker_array.markers.append(marker)

        self.trajectory_verify_publisher.publish(marker_array)



    def image_cb(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVerify()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.verify_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
