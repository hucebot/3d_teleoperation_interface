#!/usr/bin/env python

import sys, threading

from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget
from PySide6.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ros2_3d_interface.common.darkstyle import dark_style


class VideoWindowNode(Node):
    def __init__(self):
        super().__init__('video_window_node')

        self.left_gripper_sub = self.create_subscription(String, '/left_gripper', self.left_gripper_callback, 10)
        self.right_gripper_sub = self.create_subscription(String, '/right_gripper', self.right_gripper_callback, 10)
        self.main_camera_sub = self.create_subscription(String, '/main_camera', self.main_camera_callback, 10)

        self.left_gripper_status = "Left Gripper"
        self.right_gripper_status = "Right Gripper"
        self.main_camera_status = "Main Camera"

        self.get_logger().info("Video Window Node started!")

    def left_gripper_callback(self, msg):
        self.left_gripper_status = msg.data
        self.get_logger().info(f"Left Gripper: {msg.data}")

    def right_gripper_callback(self, msg):
        self.right_gripper_status = msg.data
        self.get_logger().info(f"Right Gripper: {msg.data}")

    def main_camera_callback(self, msg):
        self.main_camera_status = msg.data
        self.get_logger().info(f"Main Camera: {msg.data}")


class VideoWindowGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setWindowTitle('Video Window')
        self.setGeometry(300, 100, 1200, 800)

        self.top_left_label = QLabel("Left Gripper")
        self.top_left_label.setAlignment(Qt.AlignCenter)
        self.top_left_label.setStyleSheet("border: 1px solid black; background-color: lightgray;")

        self.top_right_label = QLabel("Right Gripper")
        self.top_right_label.setAlignment(Qt.AlignCenter)
        self.top_right_label.setStyleSheet("border: 1px solid black; background-color: lightgray;")

        self.bottom_label = QLabel("Main Camera")
        self.bottom_label.setAlignment(Qt.AlignCenter)
        self.bottom_label.setStyleSheet("border: 1px solid black; background-color: lightgray;")

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.top_left_label)
        top_layout.addWidget(self.top_right_label)

        main_layout = QVBoxLayout()
        main_layout.addLayout(top_layout)
        main_layout.addWidget(self.bottom_label)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(100)

    def update_labels(self):
        self.top_left_label.setText(self.node.left_gripper_status)
        self.top_right_label.setText(self.node.right_gripper_status)
        self.bottom_label.setText(self.node.main_camera_status)


def main(args=None):
    rclpy.init(args=args)
    node = VideoWindowNode()

    app = QApplication(sys.argv)
    dark_style(app)
    gui = VideoWindowGUI(node)
    gui.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app.exec()

    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
