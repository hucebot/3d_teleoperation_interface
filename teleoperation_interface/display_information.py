import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

class DisplayInformation(Node):
    def __init__(self):
        super().__init__('display_information_node')

        self.image_subscriber = self.create_subscription(Image, '/camera/raw_image', self.camera_image_cb, 10)
        self.markers_subscriber = self.create_subscription(MarkerArray, '/trajectory/markers', self.trajectory_markers_cb, 10)
        self.april_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.markerBorderBits = 2

        try:
            with open('/ros2_ws/src/teleoperation_interface/config/camera_calibration.yaml') as configuration:
                config_data = yaml.load(configuration, Loader=yaml.SafeLoader)

        except Exception as e:
            self.get_logger().error(f'Error reading the configuration file. {e}')
            rclpy.shutdown()

        self.left_gripper_id = 100
        self.right_gripper_id = 100

        self.circle_radius = 60

        self.left_force, self.left_torque = 0.00, 0.00
        self.right_force, self.right_torque = 0.00, 0.00
        self.circle_color = (255, 255, 255)

        self.image_publisher = self.create_publisher(Image, '/camera/information', 10)
        self.bridge = CvBridge()

        self.camera_matrix = np.array(config_data['camera_matrix']['data']).reshape((3, 3))
        self.dist_coeffs = np.array(config_data['distortion_coefficients']['data']).reshape((1, 5))

    def trajectory_markers_cb(self, msg):
        pass

    def draw_force_torque_indicator(self, cv_image, ids, corners, rvecs, tvecs):
        if ids is not None:
            for corner, marker_id, rvec, tvec in zip(corners, ids.flatten(), rvecs, tvecs):
                if marker_id == self.right_gripper_id:
                    # Calculate 3D point in marker space
                    marker_center = np.array([[0.0, 0.0, 0.0]])  # Center of the marker in marker's coordinate system

                    # Project the point to the image
                    img_pts, _ = cv2.projectPoints(marker_center, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                    center_x, center_y = int(img_pts[0][0][0]), int(img_pts[0][0][1])

                    # Draw circle and text
                    cv2.circle(cv_image, (center_x, center_y), self.circle_radius, self.circle_color, -1)

                    text = f'F: {self.right_force:.2f} - T: {self.right_torque:.2f}'
                    font_scale = 1.0
                    font_thickness = 2
                    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0]
                    text_x = center_x - text_size[0] // 2
                    text_y = center_y + text_size[1] // 2
                    cv2.putText(cv_image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness)

    def camera_image_cb(self, message):
        cv_image = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.april_dictionary)

        if ids is not None:
            # Estimate the pose of each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)  # 0.05 is the marker size in meters
            self.draw_force_torque_indicator(cv_image, ids, corners, rvecs, tvecs)

        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'imu_link'
        self.image_publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    display_information = DisplayInformation()
    rclpy.spin(display_information)
    display_information.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
