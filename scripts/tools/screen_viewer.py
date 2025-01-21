#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__('compressed_image_viewer')
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/screen_recording/compressed',
            self.listener_callback,
            10
        )
        self.subscription 

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if cv_image is not None:
            cv2.imshow('Compressed Image', cv_image)
            cv2.waitKey(1)
        else:
            self.get_logger().error("Error al decodificar la imagen")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
