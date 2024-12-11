import rclpy
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import random

class RandomMarkerArray(Node):
    def __init__(self):
        super().__init__('random_markers_array')
        
        self.marker_array_pub = self.create_publisher(MarkerArray, '/trajectory/markers', 10)
        
        self.rate = 1
        self.radius = 4.0 
        self.num_markers = 4 

        self.timer = self.create_timer(1.0 / self.rate, self.publish_markers)

        self.marker_positions = self.generate_fixed_positions()

        self.marker_id = 0

    def generate_fixed_positions(self):
        positions = [
            (1.5, 0.0, 0.2),
            (2.0, 0.0, 0.4),
            (3.5, 0.0, 0.6),
            (4.0, 0.0, 0.8)
            ]
        return positions

    def publish_markers(self):
        marker_array = MarkerArray()

        for i, position in enumerate(self.marker_positions):

            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'imu_link'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            marker_array.markers.append(marker)

        self.marker_array_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RandomMarkerArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
