#!/usr/bin/env python

import sys
import threading

from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QScrollArea, QGridLayout
from PySide6.QtCore import Qt, QTimer, Signal, QObject

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ros2_3d_interface.common.darkstyle import dark_style
from ros2_3d_interface.common.read_configuration import read_ros2_node_list, read_ros2_topic_list, read_teleop_configuration, read_robot_information
from ros2_3d_interface.common.status_led import StatusLed


class SignalManager(QObject):
    status_signal = Signal(str)


class RobotVisualizerNode(Node):
    def __init__(self, signal_manager, config_file):
        super().__init__('robot_visualizer')
        self.config_file = config_file
        self.signal_manager = signal_manager
        self.get_logger().info("Robot Visualizer Node started!")

    def get_active_nodes(self):
        return self.get_node_names()

    def get_active_topics(self):
        return [topic[0] for topic in self.get_topic_names_and_types()]

    def get_active_motors(self):
        return []

    def get_active_controllers(self):
        return []


class RobotVisualizerGUI(QMainWindow):
    def __init__(self, signal_manager, node, config_file):
        super().__init__()
        self.config_file = config_file
        self.node = node
        self.setWindowTitle(self.config_file['teleop_window']['window_title'])
        self.setGeometry(self.config_file['teleop_window']['window_x'], self.config_file['teleop_window']['window_y'],
                         self.config_file['teleop_window']['window_width'], self.config_file['teleop_window']['window_height'])

        self.node_list = read_ros2_node_list().get('ros2_node_list', []).split(' ')
        self.topic_list = read_ros2_topic_list().get('ros2_topic_list', []).split(' ')
        self.controllers = read_robot_information().get('controllers', [])
        self.motors = read_robot_information().get('motors', [])

        # Labels
        self.robot_info_label = QLabel("Robot Information")
        self.controllers_label = QLabel("Controllers")
        self.motors_label = QLabel("Motors")
        self.nodes_label = QLabel("Nodes")
        self.topics_label = QLabel("Topics")

        # Apply style to labels
        for label in [self.robot_info_label, self.controllers_label, self.motors_label, self.nodes_label, self.topics_label]:
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-weight: bold; font-size: 16px; border: 1px solid black;")

        # Robot Information Content
        self.robot_info_content = QVBoxLayout()
        self.robot_info_content.setAlignment(Qt.AlignTop)

        self.controllers_content = QVBoxLayout()
        self.controllers_content.setAlignment(Qt.AlignTop)

        self.motors_content = QVBoxLayout()
        self.motors_content.setAlignment(Qt.AlignTop)

        self.robot_info_widget = QWidget()
        robot_info_layout = QHBoxLayout()
        robot_info_layout.setAlignment(Qt.AlignTop)
        #robot_info_layout.addWidget(self.robot_info_label)

        # Add controllers and motors sections
        #robot_info_layout.addWidget(self.controllers_label)
        robot_info_layout.addLayout(self.controllers_content)

        #robot_info_layout.addWidget(self.motors_label)
        robot_info_layout.addLayout(self.motors_content)

        self.robot_info_widget.setLayout(robot_info_layout)

        # Scroll area for robot information
        self.robot_info_scroll = QScrollArea()
        self.robot_info_scroll.setWidget(self.robot_info_widget)
        self.robot_info_scroll.setWidgetResizable(True)

        # Nodes and Topics
        self.nodes_content = QScrollArea()
        self.nodes_content_widget = QWidget()
        self.nodes_content_layout = QVBoxLayout(self.nodes_content_widget)
        self.nodes_content_layout.setAlignment(Qt.AlignTop)
        self.nodes_content.setWidget(self.nodes_content_widget)
        self.nodes_content.setWidgetResizable(True)

        self.topics_content = QScrollArea()
        self.topics_content_widget = QWidget()
        self.topics_content_layout = QVBoxLayout(self.topics_content_widget)
        self.topics_content_layout.setAlignment(Qt.AlignTop)
        self.topics_content.setWidget(self.topics_content_widget)
        self.topics_content.setWidgetResizable(True)

        # Layouts for columns
        nodes_layout = QVBoxLayout()
        nodes_layout.addWidget(self.nodes_label)
        nodes_layout.addWidget(self.nodes_content)

        topics_layout = QVBoxLayout()
        topics_layout.addWidget(self.topics_label)
        topics_layout.addWidget(self.topics_content)

        main_layout = QHBoxLayout()
        main_layout.addWidget(self.robot_info_scroll)
        main_layout.addLayout(nodes_layout)
        main_layout.addLayout(topics_layout)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_columns)
        self.timer.start(1000)

        self.update_columns()

    def update_columns(self):
        # Update nodes
        active_nodes = self.node.get_active_nodes()
        for i in reversed(range(self.nodes_content_layout.count())):
            self.nodes_content_layout.itemAt(i).widget().deleteLater()
        nodes_grid_layout = QGridLayout()
        for index, node in enumerate(self.node_list):
            node_led = StatusLed("")
            node_led.setFixedWidth(20)
            node_led.set_state(1 if node in active_nodes else 0)
            node_label = QLabel(node)
            node_label.setAlignment(Qt.AlignLeft)
            nodes_grid_layout.addWidget(node_led, index, 0)
            nodes_grid_layout.addWidget(node_label, index, 1)
        nodes_grid_widget = QWidget()
        nodes_grid_widget.setLayout(nodes_grid_layout)
        self.nodes_content_layout.addWidget(nodes_grid_widget)

        # Update topics
        active_topics = self.node.get_active_topics()
        for i in reversed(range(self.topics_content_layout.count())):
            self.topics_content_layout.itemAt(i).widget().deleteLater()
        topics_grid_layout = QGridLayout()
        for index, topic in enumerate(self.topic_list):
            topic_led = StatusLed("")
            topic_led.setFixedWidth(20)
            topic_led.set_state(1 if topic in active_topics else 0)
            topic_label = QLabel(topic)
            topic_label.setAlignment(Qt.AlignLeft)
            topics_grid_layout.addWidget(topic_led, index, 0)
            topics_grid_layout.addWidget(topic_label, index, 1)
        topics_grid_widget = QWidget()
        topics_grid_widget.setLayout(topics_grid_layout)
        self.topics_content_layout.addWidget(topics_grid_widget)


        active_controllers = self.node.get_active_controllers()
        for i in reversed(range(self.controllers_content.count())):
            if i == 0:
                pass
            else:
                self.controllers_content.itemAt(i).widget().deleteLater()

        self.controllers_content.addWidget(self.controllers_label)

        for controller_type in self.controllers:
            controller_label = QLabel(controller_type)
            controller_label.setStyleSheet("font-weight: bold;")
            controller_label.setAlignment(Qt.AlignTop)
            controller_label.setAlignment(Qt.AlignLeft)
            controller_grid_layout = QGridLayout()
            self.controllers_content.addWidget(controller_label)
            row_index = 0
            for controller in self.controllers[controller_type].split(' '):
                controller_led = StatusLed("")
                controller_led.setFixedWidth(20)
                topic_led.set_state(1 if controller in active_controllers else 0)
                controller_label = QLabel(controller)
                controller_label.setAlignment(Qt.AlignTop)
                controller_label.setAlignment(Qt.AlignLeft)
                controller_grid_layout.addWidget(controller_led, row_index, 0)
                controller_grid_layout.addWidget(controller_label, row_index, 1)
                row_index += 1

            controller_grid_widget = QWidget()
            controller_grid_widget.setLayout(controller_grid_layout)

            self.controllers_content.addWidget(controller_grid_widget)

        for i in reversed(range(self.motors_content.count())):
            if i == 0:
                pass
            else:
                self.motors_content.itemAt(i).widget().deleteLater()

        self.motors_content.addWidget(self.motors_label)

        for motor_type in self.motors:
            motor_label = QLabel(motor_type)
            motor_label.setAlignment(Qt.AlignLeft)
            motor_label.setStyleSheet("font-weight: bold;")
            motor_grid_layout = QGridLayout()
            self.motors_content.addWidget(motor_label)
            row_index = 0
            for motor in self.motors[motor_type].split(' '):
                motor_led = StatusLed("")
                motor_led.setFixedWidth(20)
                motor_led.set_state(1 if motor in active_controllers else 0)
                motor_label = QLabel(motor)
                motor_label.setAlignment(Qt.AlignLeft)
                motor_grid_layout.addWidget(motor_led, row_index, 0)
                motor_grid_layout.addWidget(motor_label, row_index, 1)
                row_index += 1

            motor_grid_widget = QWidget()
            motor_grid_widget.setLayout(motor_grid_layout)

            self.motors_content.addWidget(motor_grid_widget)




def main(args=None):
    rclpy.init(args=args)
    signal_manager = SignalManager()
    config_file = read_teleop_configuration()

    node = RobotVisualizerNode(signal_manager, config_file)

    app = QApplication(sys.argv)
    dark_style(app)
    gui = RobotVisualizerGUI(signal_manager, node, config_file)
    gui.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app.exec()

    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
