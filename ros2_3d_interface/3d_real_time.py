import os
import moderngl
import numpy as np
import pyglet
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from ros2_3d_interface.utilities.camera import Camera
from ros2_3d_interface.utilities.viewer import Viewer
from ros2_3d_interface.utilities.shape import (
    ShapeGrid, ShapeFrame, ShapePyramid, ShapePointCloud, ShapeQuadTexture
)
from ros2_3d_interface.utilities.utils import (
    RotIdentity
)

class PointCloudViewerNode(Node):
    def __init__(self, screen_width=1240, screen_height=720):
        super().__init__('pointcloud_viewer_node')
        self.screen_width = screen_width
        self.screen_height = screen_height

        # Initialize camera and viewer
        self.cam = Camera()
        self.cam.setParams(45.0, (0, 0, self.screen_width, self.screen_height))
        self.cam.setTargetPos([5.0, 0.0, 0.0])
        self.cam.setEyePos([-10.0, -10.0, 5.0])
        self.viewer = Viewer(self.screen_width, self.screen_height, self.cam)

        # OpenGL context
        self.ctx = moderngl.create_context()
        self.ctx.enable(moderngl.BLEND | moderngl.DEPTH_TEST)
        self.ctx.enable(moderngl.PROGRAM_POINT_SIZE)

        # Initialize drawing primitives
        self.grid = ShapeGrid(self.ctx)
        self.frame = ShapeFrame(self.ctx)
        self.pyramid = ShapePyramid(self.ctx)
        self.cloud = ShapePointCloud(self.ctx, 640 * 480)

        # Placeholders for data
        self.frame_width = 640
        self.frame_height = 480
        self.size_points = self.frame_width * self.frame_height
        self.array_frames_xyz = np.zeros((self.size_points, 3), dtype=np.float32)
        self.array_frames_rgb = np.zeros((self.size_points, 3), dtype=np.float32)

        # Subscribe to PointCloud2 topic
        self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )
        self.get_logger().info("Point cloud viewer node initialized")

    def pointcloud_callback(self, msg):
        """Callback for PointCloud2 messages."""
        self.get_logger().info("Received point cloud message")
        try:
            points = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z', 'rgb'))

            xyz_data = np.zeros((self.size_points, 3), dtype=np.float32)
            rgb_data = np.zeros((self.size_points, 3), dtype=np.float32)

            rotation_x = np.array([[1, 0, 0],
                                       [0, 0, 1],
                                       [0, -1, 0]])
            rotation_z = np.array([[0, 1, 0],
                                    [1, 0, 0],
                                    [0, 0, 1]])

            for i, point in enumerate(points):
                if i >= self.size_points:
                    break

                x, y, z, rgb = point
                original_point = np.array([x, y, z])
                
                rotated_point = rotation_z @ (rotation_x @ original_point)
                x, y, z = rotated_point
                
                xyz_data[i] = [x, y, z]

                packed_rgb = np.frombuffer(np.array([rgb], dtype=np.float32).tobytes(), dtype=np.uint32)[0]
                r = (packed_rgb & 0x00FF0000) >> 16
                g = (packed_rgb & 0x0000FF00) >> 8
                b = (packed_rgb & 0x000000FF)
                rgb_data[i] = [r / 255.0, g / 255.0, b / 255.0]

            self.array_frames_xyz = xyz_data
            self.array_frames_rgb = rgb_data
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def update(self, dt):
        """Update and render the scene."""
        self.viewer.update(dt)

        # Update cloud points and texture
        self.cloud.update_points(array_xyz=self.array_frames_xyz)
        self.cloud.update_points(array_rgb=self.array_frames_rgb)

        # Clear and render
        self.ctx.screen.use()
        self.ctx.clear(0.3, 0.3, 0.3)

        self.grid.render(self.cam)
        self.frame.render(
            self.cam,
            pos=[0.0, 0.0, 0.0],
            rot=RotIdentity(),
            scale=1.0
        )

        far = 5.0
        fovx = 60.0
        self.cloud.render(
            self.cam,
            pos=[0.0, 0.0, 1.0],
            rot=RotIdentity(),
            scale=10.0
        )

        self.pyramid.render(
            self.cam,
            pos=[0.0, 0.0, 0.0],
            rot=RotIdentity(),
            fovx=fovx,
            aspect=self.screen_width / self.screen_height,
            far=far,
            color=[1.0, 1.0, 1.0, 1.0]
        )

    def spin_once(self):
        """Spin ROS once."""
        rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudViewerNode()

    # Schedule the ROS spin in the pyglet clock
    pyglet.clock.schedule_interval(lambda dt: node.spin_once(), 1 / 60.0)
    pyglet.clock.schedule_interval(node.update, 1 / 30.0)

    try:
        pyglet.app.run()  # This will now include ROS spinning
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
