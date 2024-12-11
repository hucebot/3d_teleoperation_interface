import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.image_publisher = self.create_publisher(Image, '/camera/raw_image', 10)
        self.bridge = CvBridge()

        try:
            self.camera = cv2.VideoCapture('/dev/video2')
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('Camera node started')
        except Exception as e:
            self.get_logger().error(f'Failed to start the camera, check connection: {e}')

        self.rate = 30
        self.camera_timmer = self.create_timer(1 / self.rate, self.camera_publisher_timmer)

    def camera_publisher_timmer(self):
        ret, frame = self.camera.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'imu_link'
            self.image_publisher.publish(image_msg)
        else:
            self.get_logger().error('Failed to read frame from camera')

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraPublisher()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    

