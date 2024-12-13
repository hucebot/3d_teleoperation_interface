import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from datetime import datetime

class PointCloudSaver(Node):
    """
    Node that subscribes to a PointCloud2 topic, saves the data dynamically with available fields to a PLY file.
    """

    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10)
        self.saved = False  # Ensure we save only one cloud and exit

    def pointcloud_callback(self, msg):
        if not self.saved:
            self.get_logger().info("Received PointCloud2 message, saving to file...")
            self.save_pointcloud(msg)
            self.saved = True
            rclpy.shutdown()  # Shutdown the node after saving the cloud

    def save_pointcloud(self, msg):
        try:
            # Extract fields dynamically
            fields = [field.name for field in msg.fields]
            points = list(pc2.read_points(msg, skip_nans=True, field_names=fields))
            
            # Save to file
            if points:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"/ros2_ws/src/ros2_3d_interface/pointcloud/pointcloud_{timestamp}.ply"
                self.write_ply_file(filename, fields, points)
                self.get_logger().info(f"Point cloud saved to {filename}")
            else:
                self.get_logger().warn("PointCloud2 message is empty.")
        except Exception as e:
            self.get_logger().error(f"Failed to process PointCloud2: {e}")

    def write_ply_file(self, filename, fields, points):
        """Write point cloud data to a PLY file dynamically based on fields."""
        try:
            with open(filename, 'w') as f:
                # Write PLY header
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points)}\n")
                
                # Define each field dynamically
                for field in fields:
                    if field == "rgb":
                        f.write("property uchar red\n")
                        f.write("property uchar green\n")
                        f.write("property uchar blue\n")
                    else:
                        f.write(f"property float {field}\n")
                
                f.write("end_header\n")
                
                # Write point data
                for point in points:
                    line = []
                    for i, field in enumerate(fields):
                        if field == "rgb":
                            # Convert float RGB to individual uchar values
                            rgb = int(point[i])  # RGB packed as a single 32-bit integer
                            r = (rgb >> 16) & 0xFF
                            g = (rgb >> 8) & 0xFF
                            b = rgb & 0xFF
                            line.extend([r, g, b])
                        else:
                            line.append(point[i])
                    f.write(" ".join(map(str, line)) + "\n")
        except Exception as e:
            self.get_logger().error(f"Failed to write PLY file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
