#!/usr/bin/env python

import os, moderngl, rclpy, pyglet, datetime
import struct

import numpy as np
import open3d as o3d

from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs_py.point_cloud2 as pc2


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

        self.render_pyramid = False

        self.screen_width = screen_width
        self.screen_height = screen_height
        self.frame_width = 640
        self.frame_height = 480
        self.size_points = self.frame_width * self.frame_height * 2

        self.array_frames_xyz = np.zeros((self.size_points, 3), dtype=np.float32)
        self.array_frames_rgb = np.zeros((self.size_points, 3), dtype=np.float32)

        self.cloud_data = np.zeros((self.size_points, 4), dtype=np.float32)
        self.rgb_buffer = np.zeros((self.size_points,), dtype=np.uint32)
        self.xyz_buffer = np.zeros((self.size_points,), dtype=np.uint32)


        self.cam = Camera()
        self.cam.setParams(45.0, (0, 0, self.screen_width, self.screen_height))
        self.cam.setTargetPos([5.0, 0.0, 0.0])
        self.cam.setEyePos([-10.0, -10.0, 5.0])
        self.viewer = Viewer(self.screen_width, self.screen_height, self.cam)

        self.ctx = moderngl.create_context()
        self.ctx.enable(moderngl.BLEND | moderngl.DEPTH_TEST)
        self.ctx.enable(moderngl.PROGRAM_POINT_SIZE)

        self.grid = ShapeGrid(self.ctx)
        self.frame = ShapeFrame(self.ctx)
        self.pyramid = ShapePyramid(self.ctx)
        self.cloud = ShapePointCloud(self.ctx, self.size_points)

        self.create_subscription(
            PointCloud2,
            '/camera/depth_registered/points',
            self.pointcloud_callback,
            10
        )

        self.pub_cloud = self.create_publisher(PointCloud2, 'republished_cloud', 10)

        self.get_logger().info("Point cloud viewer node initialized")

    def pointcloud_callback(self, msg):
        num_points = msg.width * msg.height

        cloud_arr = np.frombuffer(
            msg.data,
            dtype=[
                ("x",    np.float32),
                ("y",    np.float32),
                ("z",    np.float32),
                ("_pad", np.float32),
                ("rgb",  np.float32),
            ],
            count=num_points
        )

        xyz = np.column_stack((cloud_arr["x"], cloud_arr["y"], cloud_arr["z"]))

        republished_msg = pc2.create_cloud_xyz32(msg.header, xyz)

        self.pub_cloud.publish(republished_msg)

        rgb = cloud_arr["rgb"].view(np.uint32)
        r = (rgb & 0x00FF0000) >> 16
        g = (rgb & 0x0000FF00) >> 8
        b = (rgb & 0x000000FF)
        rgb = np.column_stack((r, g, b)).astype(np.float32) / 255.0

        self.array_frames_xyz = xyz
        self.array_frames_rgb = rgb


    def update(self, dt):
        self.viewer.update(dt)

        self.cloud.update_points(
            array_xyz=self.array_frames_xyz,
            array_rgb=self.array_frames_rgb
        )

        self.ctx.screen.use()
        self.ctx.clear(0.3, 0.3, 0.3)

        self.grid.render(self.cam)
        self.frame.render(
            self.cam,
            pos=[0.0, 0.0, 0.0],
            rot=RotIdentity(),
            scale=1.0
        )

        self.cloud.render(
            self.cam,
            pos=[0.0, 0.0, 1.0],
            rot=RotIdentity(),
            scale=10.0
        )

        if self.render_pyramid:
            self.pyramid.render(
                self.cam,
                pos=[0.0, 0.0, 0.0],
                rot=RotIdentity(),
                fovx=60.0,
                aspect=self.screen_width / self.screen_height,
                far=5.0,
                color=[1.0, 1.0, 1.0, 1.0]
            )

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)



def main(args=None):
    rclpy.init(args=args)
    node = PointCloudViewerNode()

    pyglet.clock.schedule_interval(lambda dt: node.spin_once(), 1 / 30.0)
    pyglet.clock.schedule_interval(node.update, 1 / 30.0)

    try:
        pyglet.app.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
