#!/usr/bin/env python

import sys, threading

from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PySide6.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ros2_3d_interface.common.darkstyle import dark_style

class RobotVisualizerNode(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        self.publisher = self.create_publisher(String, 'network_status', 10)
        self.robot_ip = '192.168.50.15'
        self.servers = ["8.8.8.8", "1.1.1.1", "google.com"]
        self.get_logger().info("Robot Visualizer Node started!")

class RobotVisualizerGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Robot Visualizer')
        self.setGeometry(300, 100, 1200, 800)

        self.bottom_label = QLabel("Robot Visualizer")
        self.bottom_label.setAlignment(Qt.AlignCenter)
        self.bottom_label.setStyleSheet("border: 1px solid black; background-color: lightgray;")

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.bottom_label)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_label)
        self.timer.start(1000)

        self.status_messages = []

    def update_label(self):
        if self.status_messages:
            self.bottom_label.setText("\n".join(self.status_messages[-3:]))


def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizerNode()

    app = QApplication(sys.argv)
    dark_style(app)
    gui = RobotVisualizerGUI(node)
    gui.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app.exec()

    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
