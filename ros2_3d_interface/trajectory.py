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

        self.origin = np.array([0.0, 0.0, 0.0])
        self.origin_point = Point()
        self.origin_point.x = self.origin[0]
        self.origin_point.y = self.origin[1]
        self.origin_point.z = self.origin[2]

        self.point_cloud = None

        self.trajectory_publisher = self.create_publisher(
            MarkerArray,
            '/trajectory_points',
            10
        )

        self.line_of_sight_publisher = self.create_publisher(
            MarkerArray,
            '/line_of_sight',
            10
        )

        self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )

        self.line_of_sight_markers = self.initialize_line_of_sight_markers()

    def initialize_line_of_sight_markers(self):
        line_of_sight_markers = []
        for i, point in enumerate(self.trajectory):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.005
            marker.scale.y = 0.02
            marker.scale.z = 0.05
            marker.color.a = 0.3 
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.points.append(self.origin_point)
            end_point = Point()
            end_point.x = point[0] / 10
            end_point.y = point[1] / 10
            end_point.z = point[2] / 10
            marker.points.append(end_point)

            line_of_sight_markers.append(marker)
        return line_of_sight_markers

    def publish_trajectory(self):
        marker_array = MarkerArray()
        for i, point in enumerate(self.trajectory):
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

    def pointcloud_callback(self, msg):
        self.point_cloud = np.array([
            [point[0], point[1], point[2]]
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])
        self.check_line_of_sight()
        self.publish_line_of_sight_markers()
        self.publish_trajectory()

    def check_line_of_sight(self):
        max_distance = 1.0
        point_sampling_rate = 10

        if self.point_cloud is None or len(self.point_cloud) == 0:
            return

        sampled_points = self.point_cloud[::point_sampling_rate]

        for i, point in enumerate(self.trajectory):
            trajectory_point = np.array([point[0] / 10, point[1] / 10, point[2] / 10])

            direction_vector = trajectory_point - self.origin
            distance_to_trajectory_point = np.linalg.norm(direction_vector)
            unit_vector = direction_vector / (distance_to_trajectory_point + 1e-6)

            blocked = False

            for p in sampled_points:
                point_vector = np.array(p) - self.origin
                distance_to_point = np.linalg.norm(point_vector)

                if distance_to_point <= distance_to_trajectory_point:
                    dot_product = np.dot(point_vector / (distance_to_point + 1e-6), unit_vector)
                    print(f"Dot product: {dot_product}")
                    if dot_product < 0.00:
                        blocked = True
                        break

    
            self.update_line_of_sight_marker(i, blocked)

    def update_line_of_sight_marker(self, marker_id, blocked):
        marker = self.line_of_sight_markers[marker_id]
        if blocked:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3

    def publish_line_of_sight_markers(self):
        marker_array = MarkerArray()
        for marker in self.line_of_sight_markers:
            marker.header.stamp = self.get_clock().now().to_msg()
            marker_array.markers.append(marker)
        self.line_of_sight_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
