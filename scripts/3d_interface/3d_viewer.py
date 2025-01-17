#!/usr/bin/env python

import moderngl, rclpy, pyglet

import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

from rclpy.qos import qos_profile_sensor_data

from ros2_3d_interface.utilities.camera import Camera
from ros2_3d_interface.utilities.viewer import Viewer
from ros2_3d_interface.utilities.shape import (
    ShapeGrid, 
    ShapeFrame, 
    ShapePyramid, 
    ShapePointCloud, 
    ShapeTrajectory, 
    ShapeQuadTexture,
)
from ros2_3d_interface.utilities.utils import (
    RotIdentity
)

from ros2_3d_interface.common.read_configuration import read_3d_configuration, read_general_configuration

class PointCloudViewerNode(Node):
    """
    ROS 2 Node for visualizing point clouds, trajectory markers, and RGB images in a 3D viewer.
    """

    def __init__(self, config_file):
        super().__init__('pointcloud_viewer_node')

        self.title = config_file['main_window']['name']
        self.screen_width = config_file['main_window']['width']
        self.screen_height = config_file['main_window']['height']
        self.render_hz = config_file['main_window']['render_hz']
        self.camera_velocity = config_file['main_window']['camera_velocity']

        self.frontal_view_x = config_file['viewports']['frontal']['x']
        self.frontal_view_y = config_file['viewports']['frontal']['y']
        self.frontal_view_width = config_file['viewports']['frontal']['width']
        self.frontal_view_height = config_file['viewports']['frontal']['height']
        self.frontal_camera_x = config_file['viewports']['frontal']['camera']['x_origin']
        self.frontal_camera_y = config_file['viewports']['frontal']['camera']['y_origin']
        self.frontal_camera_z = config_file['viewports']['frontal']['camera']['z_origin']
        self.frontal_camera_x_target = config_file['viewports']['frontal']['camera']['x_target']
        self.frontal_camera_y_target = config_file['viewports']['frontal']['camera']['y_target']
        self.frontal_camera_z_target = config_file['viewports']['frontal']['camera']['z_target']

        self.side_view_x = config_file['viewports']['side']['x']
        self.side_view_y = config_file['viewports']['side']['y']
        self.side_view_width = config_file['viewports']['side']['width']
        self.side_view_height = config_file['viewports']['side']['height']
        self.side_camera_x = config_file['viewports']['side']['camera']['x_origin']
        self.side_camera_y = config_file['viewports']['side']['camera']['y_origin']
        self.side_camera_z = config_file['viewports']['side']['camera']['z_origin']
        self.side_camera_x_target = config_file['viewports']['side']['camera']['x_target']
        self.side_camera_y_target = config_file['viewports']['side']['camera']['y_target']
        self.side_camera_z_target = config_file['viewports']['side']['camera']['z_target']

        self.upper_view_x = config_file['viewports']['upper']['x']
        self.upper_view_y = config_file['viewports']['upper']['y']
        self.upper_view_width = config_file['viewports']['upper']['width']
        self.upper_view_height = config_file['viewports']['upper']['height']
        self.upper_camera_x = config_file['viewports']['upper']['camera']['x_origin']
        self.upper_camera_y = config_file['viewports']['upper']['camera']['y_origin']
        self.upper_camera_z = config_file['viewports']['upper']['camera']['z_origin']
        self.upper_camera_x_target = config_file['viewports']['upper']['camera']['x_target']
        self.upper_camera_y_target = config_file['viewports']['upper']['camera']['y_target']
        self.upper_camera_z_target = config_file['viewports']['upper']['camera']['z_target']

        self.render_pyramid = config_file['rendering']['pyramid']
        self.render_trajectory = config_file['rendering']['trajectory']
        self.render_image = config_file['rendering']['image']
        self.render_robot = config_file['rendering']['robot']

        self.point_size_point_cloud = config_file['point_cloud']['point_size']
        self.depth_frame_width = config_file['point_cloud']['width']
        self.depth_frame_height = config_file['point_cloud']['height']
        self.hfov = config_file['point_cloud']['hfov']
        self.vfov = config_file['point_cloud']['vfov']
        self.point_cloud_multiplier = config_file['point_cloud']['size_multiplier']
        self.point_cloud_topic = config_file['point_cloud']['topic']
        self.size_points = self.depth_frame_width * self.depth_frame_height * self.point_cloud_multiplier

        self.rgb_image_width = config_file['rgb_image']['width']
        self.rgb_image_height = config_file['rgb_image']['height']
        self.visualizer_x = config_file['rgb_image']['visualizer']['x']
        self.visualizer_y = config_file['rgb_image']['visualizer']['y']
        self.visualizer_z = config_file['rgb_image']['visualizer']['z']
        self.visualizer_width = config_file['rgb_image']['visualizer']['width']
        self.visualizer_height = config_file['rgb_image']['visualizer']['height']
        self.image_topic = config_file['rgb_image']['topic']

        self.reset_view_topic = config_file['streamdeck']['reset_view_topic']
        
        self.trajectory_points_topic = config_file['trajectory']['topic']

        self.robot_model = config_file['robot']['model']
        self.robot_version = config_file['robot']['version']
        

        self.array_frames_xyz = np.zeros((self.size_points, 3), dtype=np.float32)
        self.array_frames_rgb = np.zeros((self.size_points, 3), dtype=np.float32)

        self.cloud_data = np.zeros((self.size_points, 4), dtype=np.float32)
        self.rgb_buffer = np.zeros((self.size_points,), dtype=np.uint32)
        self.xyz_buffer = np.zeros((self.size_points,), dtype=np.uint32)


        self.frontal_camera = Camera()
        self.frontal_camera.setParams(self.hfov, self.vfov, (0, 0, self.screen_width, self.screen_height))
        self.frontal_camera.setTargetPos([self.frontal_camera_x_target, self.frontal_camera_y_target, self.frontal_camera_z_target])
        self.frontal_camera.setEyePos([self.frontal_camera_x, self.frontal_camera_y, self.frontal_camera_z])

        self.upper_camera = Camera()
        self.upper_camera.setParams(self.hfov, self.vfov, (0, 0, self.screen_width, self.screen_height))
        self.upper_camera.setTargetPos([self.upper_camera_x_target, self.upper_camera_y_target, self.upper_camera_z_target])
        self.upper_camera.setEyePos([self.upper_camera_x, self.upper_camera_y, self.upper_camera_z])

        self.side_camera = Camera()
        self.side_camera.setParams(self.hfov, self.vfov, (0, 0, self.screen_width, self.screen_height))
        self.side_camera.setTargetPos([self.side_camera_x_target, self.side_camera_y_target, self.side_camera_z_target])
        self.side_camera.setEyePos([self.side_camera_x, self.side_camera_y, self.side_camera_z])

        self.viewer = Viewer(
            screen_width=self.screen_width, 
            screen_height=self.screen_height, 
            cam=self.frontal_camera, 
            title=self.title,
            cam_velocity=self.camera_velocity
        )

        self.view_type = ''

        self.ctx = moderngl.create_context()
        self.ctx.enable(moderngl.BLEND | moderngl.DEPTH_TEST)
        self.ctx.enable(moderngl.PROGRAM_POINT_SIZE)

        self.grid = ShapeGrid(self.ctx)
        self.frame = ShapeFrame(self.ctx)
        self.pyramid = ShapePyramid(self.ctx)
        self.cloud = ShapePointCloud(self.ctx, self.size_points, point_size=self.point_size_point_cloud)
        self.image = ShapeQuadTexture(self.ctx, self.rgb_image_width, self.rgb_image_height)

        self.actual_image = np.zeros((self.rgb_image_height, self.rgb_image_width, 3), dtype=np.uint8)

        self.trajectory_points = []
        self.trajectory = None


        self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self.pointcloud_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            MarkerArray,
            self.trajectory_points_topic,
            self.trajectory_cb,
            10
        )

        self.create_subscription(
            Image,
            self.image_topic,
            self.camera_image_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            String,
            self.reset_view_topic,
            self.camera_view_cb,
            10
        )

        self.get_logger().info("3D Viewer is Ready")

    def camera_view_cb(self, msg):
        """
        Callback for receiving camera view commands.

        Args:
            msg (std_msgs.msg.String): The incoming camera view command.

        This function updates the camera position and target based on the received command.
        """
        self.frontal_camera.setEyePos(np.array([self.frontal_camera_x, self.frontal_camera_y, self.frontal_camera_z], dtype=np.float32))
        self.frontal_camera.setTargetPos(np.array([self.frontal_camera_x_target, self.frontal_camera_y_target, self.frontal_camera_z_target], dtype=np.float32))



    def camera_image_cb(self, msg):
        """
        Callback for receiving RGB image data.


        Args:
            msg (sensor_msgs.msg.Image): The incoming RGB image message.

        This function converts the image data from the ROS message into a numpy array for rendering
        in the 3D viewer.
        """

        self.actual_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

    def trajectory_cb(self, msg):
        """
        Callback for receiving trajectory points as MarkerArray.

        Args:
            msg (visualization_msgs.msg.MarkerArray): The incoming trajectory points.

        This function extracts the trajectory points from the message and updates the `ShapeTrajectory`
        object for rendering in the viewer.
        """


        self.trajectory_points = []
        for marker in msg.markers:
            self.trajectory_points.append([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]) 
        self.trajectory = ShapeTrajectory(self.ctx, self.trajectory_points, color=[0.0, 1.0, 0.0, 0.5])

    def pointcloud_cb(self, msg):
        """
        Callback for receiving point cloud data.

        Args:
            msg (sensor_msgs.msg.PointCloud2): The incoming point cloud message.

        This function extracts XYZ coordinates and RGB values from the point cloud message and updates
        the arrays used for rendering the point cloud in the 3D viewer.
        """
        
        try:
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

            rgb = cloud_arr["rgb"].view(np.uint32)
            r = (rgb & 0x00FF0000) >> 16
            g = (rgb & 0x0000FF00) >> 8
            b = (rgb & 0x000000FF)
            rgb = np.column_stack((r, g, b)).astype(np.float32) / 255.0

            self.array_frames_xyz = xyz
            self.array_frames_rgb = rgb

        except Exception as e:
            self.get_logger().error("Error processing pointcloud: %s" % str(e))

    def draw_scene(self, camera, render_trajectory = True):
        try:
            self.grid.render(
                cam=camera
            )
            self.frame.render(
                cam=camera,
                pos=[0.0, 0.0, -1.0],
                rot=RotIdentity(),
                scale=0.3
            )

            if self.trajectory is not None and render_trajectory:
                self.trajectory.render(cam=camera)

            self.cloud.render(
                cam=camera,
                pos=[0.0, 0.0, 0.0],
                rot=RotIdentity(),
                scale=1.0
            )

            if self.render_image:
                self.image.update_texture(
                    self.actual_image)

                self.image.render(
                    cam=camera, 
                    pos=[self.visualizer_x, self.visualizer_y, self.visualizer_z],
                    rot=RotIdentity(), 
                    size=[self.visualizer_width, self.visualizer_height]
                )

            if self.render_pyramid:
                self.pyramid.render(
                    cam=camera,
                    pos=[0.0, 0.0, 0.0],
                    rot=RotIdentity(),
                    fovx=60.0,
                    aspect=self.screen_width / self.screen_height,
                    far=5.0,
                    color=[1.0, 1.0, 1.0, 1.0]
                )

        except Exception as e:
            self.get_logger().error("Error updating frontal viewer: %s" % str(e))

    def update(self, dt):
        """
        Updates the 3D viewer, rendering the current scene.

        Args:
            dt (float): The time elapsed since the last update.

        This function handles the rendering of the 3D scene's components.
        """
        self.viewer.update(dt)

        self.cloud.update_points(
            array_xyz=self.array_frames_xyz,
            array_rgb=self.array_frames_rgb
        )

        self.ctx.screen.use()
        self.ctx.clear(0.3, 0.3, 0.3)

        self.ctx.viewport = (
            self.frontal_view_x,
            self.frontal_view_y,
            self.frontal_view_width,
            self.frontal_view_height
        )
        self.draw_scene(self.frontal_camera, render_trajectory = True)

        self.ctx.viewport = (
            self.side_view_x,
            self.side_view_y,
            self.side_view_width,
            self.side_view_height
        )
        self.draw_scene(self.side_camera, render_trajectory = True)

        self.ctx.viewport = (
            self.upper_view_x,
            self.upper_view_y,
            self.upper_view_width,
            self.upper_view_height
        )
        self.draw_scene(self.upper_camera, render_trajectory = True)

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):    
    rclpy.init(args=args)
    node = PointCloudViewerNode(config_file=read_3d_configuration())

    pyglet.clock.schedule_interval(lambda dt: node.spin_once(), 1 / 60)
    pyglet.clock.schedule_interval(node.update, 1 / 60)

    try:
        pyglet.app.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()