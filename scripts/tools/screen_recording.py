#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import mss

from ros2_3d_interface.common.read_configuration import read_teleop_configuration, read_3d_configuration


class ScreenRecording(Node):
    def __init__(self):
        super().__init__('screen_recording')
        self.config_data = read_teleop_configuration()
        self.config_3d_data = read_3d_configuration()

        self.screen_publisher = self.create_publisher(CompressedImage, self.config_data['screen_recording']['topic'], 10)

        fps = self.config_data['screen_recording']['fps']
        self.timer = self.create_timer(1.0/fps, self.timer_screen_cb)

        self.x = self.config_3d_data['main_window']['x']
        self.y = self.config_3d_data['main_window']['y']
        self.width = self.config_3d_data['main_window']['width']
        self.height = self.config_3d_data['main_window']['height']

        self.monitor = {
            "top": 0,
            "left": 0,
            "width": self.width,
            "height": self.height
        }

        self.sct = mss.mss()

    def timer_screen_cb(self):
        img_bgra = np.array(self.sct.grab(self.monitor))
        img_bgr = img_bgra[:, :, :3]
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        success, encoded_img = cv2.imencode('.jpg', img_bgr, encode_param)
        if not success:
            self.get_logger().error("Error encoding image")
            return

        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.header.frame_id = "screen_recording"
        compressed_msg.format = "jpeg"
        compressed_msg.data = encoded_img.tobytes()

        self.screen_publisher.publish(compressed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScreenRecording()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
