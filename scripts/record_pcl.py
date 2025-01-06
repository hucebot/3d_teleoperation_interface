#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
from datetime import datetime
import numpy as np
from pynput import keyboard
from threading import Thread, Lock
import struct
from tqdm import tqdm


class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10)

        self.recording = False
        self.lock = Lock()
        self.raw_frames = []
        self.frame_width = 640
        self.frame_height = 480
        self.frame_counter = 0

        self.xyz_frames = []
        self.rgb_frames = []

        self.keyboard_thread = Thread(target=self.run_keyboard_listener, daemon=True)
        self.keyboard_thread.start()


    def pointcloud_callback(self, msg):
        points = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z', 'rgb'))

        if self.recording:
            with self.lock:
                if self.frame_counter % 2 == 0:
                    self.raw_frames.append(msg)
                    self.get_logger().info(f"Frame saved. Total saved frames: {len(self.raw_frames)}")
                self.frame_counter += 1

    def process_and_save_frames(self):
        try:
            self.get_logger().info("Processing frames...")
            for msg in tqdm(self.raw_frames, desc="Processing frames", unit="frame"):
                points = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z', 'rgb'))
                
                xyz_frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.float32)
                rgb_frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.float32)

                rotation_x = np.array([[1, 0, 0],
                                       [0, 0, 1],
                                       [0, -1, 0]])
                rotation_z = np.array([[0, 1, 0],
                                       [1, 0, 0],
                                       [0, 0, 1]])

                for i, point in enumerate(points):
                    if i >= self.frame_width * self.frame_height:
                        break

                    x, y, z, rgb = point
                    row = i // self.frame_width
                    col = i % self.frame_width

                    original_point = np.array([x, y, z])
                    rotated_point = rotation_z @ (rotation_x @ original_point)

                    xyz_frame[row, col] = rotated_point

                    #packed_rgb = struct.unpack('I', struct.pack('f', rgb))[0]
                    packed_rgb = np.frombuffer(np.array([rgb], dtype=np.float32).tobytes(), dtype=np.uint32)[0]

                    r = (packed_rgb & 0x00FF0000) >> 16
                    g = (packed_rgb & 0x0000FF00) >> 8
                    b = (packed_rgb & 0x000000FF)
                    
                    rgb_frame[row, col] = [r / 255.0, g / 255.0, b / 255.0]

                self.xyz_frames.append(xyz_frame)
                self.rgb_frames.append(rgb_frame)

            self.save_to_file(self.xyz_frames, self.rgb_frames)

        except Exception as e:
            self.get_logger().error(f"Failed to process frames: {e}")
            pass

    def save_to_file(self, xyz_frames, rgb_frames):
        try:
            xyz_array = np.stack(xyz_frames, axis=0)
            rgb_array = np.stack(rgb_frames, axis=0)

            num_frames = len(xyz_frames)
            
            filepath = f"/ros2_ws/src/ros2_3d_interface/pointclouds/pointcloud_0000_1.npz"
            np.savez(filepath, xyz=xyz_array, color=rgb_array)
            self.get_logger().info(f"Saved {num_frames} frames to {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save file: {e}")

    def on_press(self, key):
        try:
            if key == keyboard.Key.space:
                if not self.recording:
                    self.recording = True
                    self.raw_frames = []
                    self.frame_counter = 0
                    self.get_logger().info("Recording started.")
            elif key.char == 's':
                if self.recording:
                    self.recording = False
                    self.get_logger().info("Recording stopped. Processing frames...")
                    self.process_and_save_frames()
        except AttributeError:
            pass

    def run_keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
