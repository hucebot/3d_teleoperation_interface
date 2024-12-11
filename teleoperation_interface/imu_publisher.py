import rclpy
import serial
import struct
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation

class WitmotionImu(Node):
    def __init__(self):
        super().__init__('witmotion_imu')
        
        self.imu_pub = self.create_publisher(Imu, '/witmotion/sensor/imu', 10)
        self.marker_pub = self.create_publisher(Marker, '/witmotion/marker', 10)
        
        try:
            self.serial_device = serial.Serial('/dev/ttyUSB0', 115200)
            self.get_logger().info('Witmotion IMU node started')
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            rclpy.shutdown()

        self.rate = 30
        self.first_angle_flag = False
        self.initial_angle = np.zeros(3)
        self.timer = self.create_timer(1.0 / self.rate, self.read_data)

    def euler_to_quaternion(self, roll, pitch, yaw):
        original_rot = Rotation.from_euler('xyz', [pitch, -roll, -yaw], degrees=True)
        correction_rot = Rotation.from_euler('x', 180, degrees=True)
        corrected_rot = correction_rot * original_rot
        q = corrected_rot.as_quat()
        q = np.round(q, 3)
        return q[::-1]

    def publish_marker(self, q):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'witmotion_imu'
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.scale.x = 1.0 
        marker.scale.y = 0.3
        marker.scale.z = 0.3 
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def read_data(self):
        try:
            data = self.serial_device.read_until(b'U')
            if data[0] == 0x61 and len(data) == 20:
                # Acceleration data
                acc = np.array(struct.unpack('<hhh', data[1:7])) / 32768.0 * 16.0 * 9.8
                acc = np.round(acc, 3)

                # Angular velocity data
                gyro = np.array(struct.unpack('<hhh', data[7:13])) / 32768.0 * 2000.0
                gyro = np.round(gyro, 3)
                
                # Angle data
                angle = np.array(struct.unpack('<hhh', data[13:19])) / 32768.0 * 180.0
                angle = np.round(angle, 3)

                angle[1] = -angle[1]

                # Capture initial angles
                if not self.first_angle_flag:
                    self.initial_angle = angle
                    self.first_angle_flag = True

                # Calculate relative angles
                relative_angle = angle - self.initial_angle

                # Convert relative angles to quaternion
                q = self.euler_to_quaternion(relative_angle[0], relative_angle[1], relative_angle[2])

                # Publish IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
                imu_msg.angular_velocity.x = gyro[0]
                imu_msg.angular_velocity.y = gyro[1]
                imu_msg.angular_velocity.z = gyro[2]
                imu_msg.linear_acceleration.x = acc[0]
                imu_msg.linear_acceleration.y = acc[1]
                imu_msg.linear_acceleration.z = acc[2]
                
                self.imu_pub.publish(imu_msg)
                self.publish_marker(q)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    witmotion_imu = WitmotionImu()
    rclpy.spin(witmotion_imu)
    witmotion_imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()