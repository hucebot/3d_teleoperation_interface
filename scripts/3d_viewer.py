#!/usr/bin/env python

import moderngl, rclpy, pyglet

import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool

from rclpy.qos import qos_profile_sensor_data

from ros2_3d_interface.utilities.camera import Camera
from ros2_3d_interface.utilities.viewer import Viewer
from ros2_3d_interface.utilities.shape import (
    ShapeGrid, 
    ShapeFrame, 
    ShapePyramid, 
    ShapePointCloud, 
    ShapeTrajectory, 
    ShapeQuadTexture
)
from ros2_3d_interface.utilities.utils import (
    RotIdentity
)

class PointCloudViewerNode(Node):
    """
    ROS 2 Node for visualizing point clouds, trajectory markers, and RGB images in a 3D viewer.
    """

    def __init__(self):
        super().__init__('pointcloud_viewer_node')

        self.declare_parameter('window_name', '3D Viewer')
        self.declare_parameter('window_width', 640)
        self.declare_parameter('window_height', 480)
        self.declare_parameter('camera_x', 0.03886509)
        self.declare_parameter('camera_y', 0.00772565)
        self.declare_parameter('camera_z', 0.00450755)
        self.declare_parameter('rgb_image_width', 640)
        self.declare_parameter('rgb_image_height', 480)
        self.declare_parameter('visualizer_x', 0.65)
        self.declare_parameter('visualizer_y', 0.54)
        self.declare_parameter('visualizer_z', -0.30)
        self.declare_parameter('visualizer_width', 0.3)
        self.declare_parameter('visualizer_height', 0.20)
        self.declare_parameter('point_cloud_width', 640)
        self.declare_parameter('point_cloud_height', 480)
        self.declare_parameter('point_cloud_size_multiplier', 2)
        self.declare_parameter('point_size', 1)
        self.declare_parameter('render_pyramid', False)
        self.declare_parameter('render_trajectory', True)
        self.declare_parameter('render_image', True)
        self.declare_parameter('render_robot', True)
        self.declare_parameter('robot_model', 'H1')
        self.declare_parameter('robot_version', 'with_hand')
        self.declare_parameter('hfov', 90)
        self.declare_parameter('vfov', 90)
        self.declare_parameter('render_hz', 60)
        self.declare_parameter('camera_velocity', 1.0)
        self.declare_parameter('point_cloud_topic', '/camera/depth_registered/points')
        self.declare_parameter('trajectory_points_topic', '/trajectory_points')
        self.declare_parameter('rgb_image_topic', '/camera/color/image_raw')
        self.declare_parameter('reset_view_topic', '/streamdeck/reset_view')

        self.render_pyramid = self.get_parameter('render_pyramid').get_parameter_value().bool_value
        self.render_trajectory = self.get_parameter('render_trajectory').get_parameter_value().bool_value
        self.render_image = self.get_parameter('render_image').get_parameter_value().bool_value
        self.render_robot = self.get_parameter('render_robot').get_parameter_value().bool_value

        self.point_size_point_cloud = self.get_parameter('point_size').get_parameter_value().integer_value

        self.screen_width = self.get_parameter('window_width').get_parameter_value().integer_value
        self.screen_height = self.get_parameter('window_height').get_parameter_value().integer_value
        
        self.rgb_image_width = self.get_parameter('rgb_image_width').get_parameter_value().integer_value
        self.rgb_image_height = self.get_parameter('rgb_image_height').get_parameter_value().integer_value
        self.visualizer_x = self.get_parameter('visualizer_x').get_parameter_value().double_value
        self.visualizer_y = self.get_parameter('visualizer_y').get_parameter_value().double_value
        self.visualizer_z = self.get_parameter('visualizer_z').get_parameter_value().double_value
        self.visualizer_width = self.get_parameter('visualizer_width').get_parameter_value().double_value
        self.visualizer_height = self.get_parameter('visualizer_height').get_parameter_value().double_value

        self.depth_frame_width = self.get_parameter('point_cloud_width').get_parameter_value().integer_value
        self.depth_frame_height = self.get_parameter('point_cloud_height').get_parameter_value().integer_value

        self.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        
        self.hfov = self.get_parameter('hfov').get_parameter_value().integer_value
        self.vfov = self.get_parameter('vfov').get_parameter_value().integer_value

        self.size_points = self.depth_frame_width * self.depth_frame_height * self.get_parameter('point_cloud_size_multiplier').get_parameter_value().integer_value

        self.array_frames_xyz = np.zeros((self.size_points, 3), dtype=np.float32)
        self.array_frames_rgb = np.zeros((self.size_points, 3), dtype=np.float32)

        self.cloud_data = np.zeros((self.size_points, 4), dtype=np.float32)
        self.rgb_buffer = np.zeros((self.size_points,), dtype=np.uint32)
        self.xyz_buffer = np.zeros((self.size_points,), dtype=np.uint32)

        self.cam = Camera()

        self.camera_x = self.get_parameter('camera_x').get_parameter_value().double_value
        self.camera_y = self.get_parameter('camera_y').get_parameter_value().double_value
        self.camera_z = self.get_parameter('camera_z').get_parameter_value().double_value

        self.cam.setParams(self.hfov, self.vfov, (0, 0, self.screen_width, self.screen_height))
        self.cam.setTargetPos([1.0, 0.0, 0.0])
        self.cam.setEyePos([self.camera_x, self.camera_y, self.camera_z])
        self.viewer = Viewer(
            screen_width=self.screen_width, 
            screen_height=self.screen_height, 
            cam=self.cam, 
            title=self.get_parameter('window_name').get_parameter_value().string_value,
            cam_velocity=self.get_parameter('camera_velocity').get_parameter_value().double_value
        )

        self.viewer.on_reset_camera = self.reset_camera

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

        self.initial_eye = np.array([self.camera_x, self.camera_y, self.camera_z], dtype=np.float32)
        self.initial_target = np.array([1.0, 0.0, 0.0], dtype=np.float32)


        self.create_subscription(
            PointCloud2,
            self.get_parameter('point_cloud_topic').get_parameter_value().string_value,
            self.pointcloud_cb,
            10
        )

        self.create_subscription(
            MarkerArray,
            self.get_parameter('trajectory_points_topic').get_parameter_value().string_value,
            self.trajectory_cb,
            10
        )

        self.create_subscription(
            Image,
            self.get_parameter('rgb_image_topic').get_parameter_value().string_value,
            self.camera_image_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Bool,
            self.get_parameter('reset_view_topic').get_parameter_value().string_value,
            lambda msg: self.reset_camera(),
            10
        )

        self.get_logger().info("3D Viewer is Ready")

    def reset_camera(self):
        """
        Resets the camera position and target to their initial values.
        """

        self.cam.setEyePos(self.initial_eye)
        self.cam.setTargetPos(self.initial_target)

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

    def update(self, dt):
        """
        Updates the 3D viewer, rendering the current scene.

        Args:
            dt (float): The time elapsed since the last update.

        This function handles the rendering of the 3D scene's components.
        """

        try:
            self.viewer.update(dt)
            
            #self.get_logger().info(f'Camera position: {self.cam.getEyePos()}')

            self.cloud.update_points(
                array_xyz=self.array_frames_xyz,
                array_rgb=self.array_frames_rgb
            )

            self.ctx.screen.use()
            self.ctx.clear(0.3, 0.3, 0.3)

            self.grid.render(
                self.cam
            )
            self.frame.render(
                self.cam,
                pos=[0.0, 0.0, -1.0],
                rot=RotIdentity(),
                scale=0.3
            )

            if self.trajectory is not None and self.render_trajectory:
                self.trajectory.render(self.cam)

            self.cloud.render(
                self.cam,
                pos=[0.0, 0.0, 0.0],
                rot=RotIdentity(),
                scale=1.0
            )

            if self.render_image:
                self.image.update_texture(
                    self.actual_image)

                self.image.render(
                    self.cam, 
                    pos=[self.visualizer_x, self.visualizer_y, self.visualizer_z],
                    rot=RotIdentity(), 
                    size=[self.visualizer_width, self.visualizer_height]
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

        except Exception as e:
            self.get_logger().error("Error updating viewer: %s" % str(e))

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):    
    rclpy.init(args=args)
    node = PointCloudViewerNode()

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
