#!/usr/bin/env python

import os
import moderngl
import numpy as np
import pyglet
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs_py.point_cloud2 as pc2
import torch

from ros2_3d_interface.utilities.camera import Camera
from ros2_3d_interface.utilities.viewer import Viewer
from ros2_3d_interface.utilities.shape import (
    ShapeGrid, ShapeFrame, ShapePyramid, ShapePointCloud, ShapeTrajectory
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
        self.update_pointcloud = False

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
        self.cloud = ShapePointCloud(self.ctx, 680*480*1)

        # Placeholders for data
        self.frame_width = 640
        self.frame_height = 480
        self.size_points = self.frame_width * self.frame_height

        self.array_frames_xyz = torch.zeros((640 * 480, 3), dtype=torch.float32, device="cuda")
        self.array_frames_rgb = torch.zeros((640 * 480, 3), dtype=torch.float32, device="cuda")


        self.trajectory_points = []
        self.trajectory = None

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

        self.create_subscription(
            Bool,
            '/update_point_cloud',
            self.update_pointcloud_callback,
            10
        )

        self.get_logger().info("Point cloud viewer node initialized")

    def update_pointcloud_callback(self, msg):
        self.update_pointcloud = msg.data

    def trajectory_callback(self, msg):
        self.trajectory_points = []
        for marker in msg.markers:
            self.trajectory_points.append([marker.pose.position.x*10, marker.pose.position.y*10, marker.pose.position.z*10]) 

        self.trajectory = ShapeTrajectory(self.ctx, self.trajectory_points, color=[0.0, 1.0, 0.0, 0.5])


    def pointcloud_callback(self, msg):
        if self.update_pointcloud:
            self.update_pointcloud = False
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
                self.get_logger().error(f"Error processing point cloud: {e}")


    def update(self, dt):
        self.viewer.update(dt)

        self.cloud.update_points(array_xyz=self.array_frames_xyz.cpu().numpy())
        self.cloud.update_points(array_rgb=self.array_frames_rgb.cpu().numpy())

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
        rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudViewerNode()

    pyglet.clock.schedule_interval(lambda dt: node.spin_once(), 1 / 60.0)
    pyglet.clock.schedule_interval(node.update, 1 / 30.0)

    try:
        pyglet.app.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
