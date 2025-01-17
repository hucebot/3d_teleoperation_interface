import rclpy
from rclpy.node import Node
import pyspacemouse

from std_msgs.msg import Float64

class Insta360Controller(Node):
    def __init__(self):
        super().__init__('insta_360_controller')

        self.get_logger().info('Insta360 Controller Node Started')
        
        self.pan_publisher = self.create_publisher(Float64, '/insta360/pan', 10)
        self.tilt_publisher = self.create_publisher(Float64, '/insta360/tilt', 10)
        
        self.pan_camera = 0.0
        self.tilt_camera = 0.0
        self.dead_zone = 0.2

        self.step_size = 0.8

        try:
            self.spacemouse = pyspacemouse.open()
            self.get_logger().info('SpaceMouse device connected successfully')
        except Exception as e:
            self.get_logger().error(f'No SpaceMouse device found. Check connection: {e}')
            self.spacemouse = None

        self.timer = self.create_timer(0.01, self.read_spacemouse)

    def read_spacemouse(self):
        if not self.spacemouse:
            return

        state = self.spacemouse.read()
        if state:
            x_mouse = state.roll
            y_mouse = state.pitch

            if abs(x_mouse) > self.dead_zone or abs(y_mouse) > self.dead_zone:
                self.pan_camera += self.step_size * x_mouse
                self.tilt_camera += self.step_size * y_mouse

                self.pan_camera = max(min(self.pan_camera, 50.0), -50.0) 
                self.tilt_camera = max(min(self.tilt_camera, 50.0), -50.0)

                pan_msg = Float64()
                pan_msg.data = self.pan_camera
                self.pan_publisher.publish(pan_msg)

                tilt_msg = Float64()
                tilt_msg.data = self.tilt_camera

                self.get_logger().info(f'Pan: {self.pan_camera}, Tilt: {self.tilt_camera}')
                self.tilt_publisher.publish(tilt_msg)
                self.pan_publisher.publish(pan_msg)

def main(args=None):
    rclpy.init(args=args)
    insta360_controller = Insta360Controller()
    rclpy.spin(insta360_controller)
    insta360_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()