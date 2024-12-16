import os
import moderngl
import numpy as np
import pyglet

from viewer_opengl.viewer import Viewer
from viewer_opengl.camera import Camera
from viewer_opengl.shape import (
    ShapeGrid, ShapeFrame, ShapePyramid, ShapePointCloud, ShapeQuadTexture
)
from viewer_opengl.utils import (
    RotIdentity
)

class PointCloudViewer:
    def __init__(self, screen_width=1800, screen_height=1200, fps=5.5):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.fps = fps

        # Initialize camera and viewer
        self.cam = Camera()
        self.cam.setParams(45.0, (0, 0, self.screen_width, self.screen_height))
        self.cam.setTargetPos([5.0, 0.0, 0.0])
        self.cam.setEyePos([-10.0, -10.0, 5.0])
        self.viewer = Viewer(self.screen_width, self.screen_height, self.cam)

        # Initialize OpenGL context
        self.ctx = moderngl.create_context()
        self.ctx.enable(moderngl.BLEND | moderngl.DEPTH_TEST)
        self.ctx.enable(moderngl.PROGRAM_POINT_SIZE)

        # Initialize drawing primitives
        self.grid = ShapeGrid(self.ctx)
        self.frame = ShapeFrame(self.ctx)
        self.pyramid = ShapePyramid(self.ctx)
        self.cloud = None
        self.quad = None

        # Load data placeholders
        self.array_frames_rgb = None
        self.array_frames_xyz = None
        self.size_batch = 0
        self.frame_width = 0
        self.frame_height = 0
        self.frame_aspect = 0
        self.size_points = 0

        # Frame indices
        self.index_frame = 0
        self.index_iterations = 0

    def load_data(self, path_file, name_dataset_rgb='color', name_dataset_xyz='xyz'):
        with np.load(path_file, allow_pickle=True) as f:
            if name_dataset_rgb in f:
                self.array_frames_rgb = f[name_dataset_rgb][()]
                assert len(self.array_frames_rgb.shape) == 4 and self.array_frames_rgb.shape[3] == 3
                print("Loaded color image:", name_dataset_rgb, self.array_frames_rgb.shape)

            if name_dataset_xyz in f:
                self.array_frames_xyz = f[name_dataset_xyz][()]
                assert len(self.array_frames_xyz.shape) == 4 and self.array_frames_xyz.shape[3] == 3
                print("Loaded point cloud:", name_dataset_xyz, self.array_frames_xyz.shape)

        if self.array_frames_rgb is None and self.array_frames_xyz is None:
            raise ValueError("No dataset loaded")

        # Retrieve image sizes
        if self.array_frames_rgb is not None:
            self.size_batch = self.array_frames_rgb.shape[0]
            self.frame_width = self.array_frames_rgb.shape[2]
            self.frame_height = self.array_frames_rgb.shape[1]

        if self.array_frames_xyz is not None:
            self.size_batch = self.array_frames_xyz.shape[0]
            self.frame_width = self.array_frames_xyz.shape[2]
            self.frame_height = self.array_frames_xyz.shape[1]

        if self.array_frames_rgb is not None and self.array_frames_xyz is not None:
            assert self.array_frames_rgb.shape == self.array_frames_xyz.shape

        self.frame_aspect = self.frame_width / self.frame_height
        self.size_points = self.frame_width * self.frame_height

        print("Loaded frames:", self.size_batch)
        print("Frames shape:", self.frame_width, self.frame_height)

        # Initialize shapes
        self.cloud = ShapePointCloud(self.ctx, self.size_points)
        self.quad = ShapeQuadTexture(self.ctx, self.frame_width, self.frame_height)

    def update(self, dt):
        self.index_iterations += 1
        self.viewer.update(dt)

        # Update frame index based on FPS
        if self.index_iterations % int(self.fps) == 0:
            self.index_frame += 1
            if self.index_frame >= self.size_batch:
                self.index_frame = 0

        # Update cloud points and texture
        if self.array_frames_xyz is not None:
            self.cloud.update_points(array_xyz=self.array_frames_xyz[self.index_frame])

        if self.array_frames_rgb is not None:
            self.cloud.update_points(array_rgb=self.array_frames_rgb[self.index_frame])
            self.quad.update_texture(self.array_frames_rgb[self.index_frame])

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
        if self.array_frames_xyz is not None:
            self.cloud.render(
                self.cam,
                pos=[0.0, 0.0, 1.0],
                rot=RotIdentity(),
                scale=10.0
            )

        if self.array_frames_rgb is not None:
            self.pyramid.render(
                self.cam,
                pos=[0.0, 0.0, 0.0],
                rot=RotIdentity(),
                fovx=fovx,
                aspect=self.frame_aspect,
                far=far,
                color=[1.0, 1.0, 1.0, 1.0]
            )

    def run(self):
        pyglet.clock.schedule_interval(self.update, 1 / 60.0)
        pyglet.app.run()

if __name__ == "__main__":
    viewer = PointCloudViewer()
    viewer.load_data('/ros2_ws/src/ros2_3d_interface/pointclouds/pointcloud_0000_2.npz')
    viewer.run()
