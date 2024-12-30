import os
import moderngl
import numpy as np
import pyglet
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs_py.point_cloud2 as pc2
import torch

from ros2_3d_interface.utilities.camera import Camera
from ros2_3d_interface.utilities.viewer import Viewer
from ros2_3d_interface.utilities.shape import (
    ShapeGrid, ShapeFrame, ShapePyramid, ShapePointCloud, ShapeQuadTexture, ShapeTrajectory
)

from ros2_3d_interface.utilities.utils import (
    RotIdentity
)

class PointCloudViewerNode(Node):
    def __init__(self, screen_width=1240, screen_height=720):
        super().__init__('pointcloud_viewer_node')
        self.screen_width = screen_width
        self.screen_height = screen_height

        self.render_pyramid = False

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

        self.array_frames_xyz = torch.zeros((640 * 480, 3), dtype=torch.float32, device="cuda")
        self.array_frames_rgb = torch.zeros((640 * 480, 3), dtype=torch.float32, device="cuda")


        self.trajectory_points = []
        self.trajectory = None

        # Subscribe to PointCloud2 topic
        self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )

        self.create_subscription(
            MarkerArray,
            '/trajectory_points',
            self.trajectory_callback,
            10
        )

        self.get_logger().info("Point cloud viewer node initialized")

    def trajectory_callback(self, msg):
        self.trajectory_points = []
        for marker in msg.markers:
            self.trajectory_points.append([marker.pose.position.x * 10, marker.pose.position.y * 10, marker.pose.position.z * 10]) 
        self.trajectory = ShapeTrajectory(self.ctx, self.trajectory_points)
        self.trajectory_color = [0.0, 1.0, 0.0, 1.0]

    def pointcloud_callback(self, msg):
        """Callback optimizado para PointCloud2."""
        try:
            cloud_data = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            points = np.array(list(cloud_data), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
            self.array_frames_xyz = torch.tensor(
                np.stack((points['x'], points['y'], points['z']), axis=-1),
                device="cuda"
            )
            rgb = np.frombuffer(points['rgb'].tobytes(), dtype=np.uint32)
            r = (rgb & 0x00FF0000) >> 16
            g = (rgb & 0x0000FF00) >> 8
            b = (rgb & 0x000000FF)
            self.array_frames_rgb = torch.tensor(
                np.stack((r, g, b), axis=-1) / 255.0,
                device="cuda"
            )
        except Exception as e:
            self.get_logger().error(f"Error procesando la nube de puntos: {e}")


    def update(self, dt):
        """Update and render the scene."""
        self.viewer.update(dt)

        # Update cloud points and texture
        self.cloud.update_points(array_xyz=self.array_frames_xyz.cpu().numpy())
        self.cloud.update_points(array_rgb=self.array_frames_rgb.cpu().numpy())

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

        if self.trajectory is not None:
            self.trajectory.render(self.cam)

        if self.render_pyramid:
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
