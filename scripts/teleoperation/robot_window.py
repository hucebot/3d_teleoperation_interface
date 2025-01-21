#!/usr/bin/env python

import sys, threading, subprocess, os, time
from collections import deque

from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QScrollArea, QGridLayout
from PySide6.QtCore import Qt, QTimer, Signal, QObject

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int16, Float32
from sensor_msgs.msg import PointCloud2

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

        self.timestamps = {}
        self.topic_list = read_ros2_topic_list().get('ros2_topic_list', []).split(' ')
        for topic in self.topic_list:
            topic_name = topic.split(",")[0]
            self.timestamps[topic_name] = deque(maxlen=100)

        for topic in self.topic_list:
            topic_name = topic.split(",")[0]
            topic_type = topic.split(",")[1]
            if topic_type == "std_msgs/msg/String":
                topic_type = String
            elif topic_type == "std_msgs/msg/Bool":
                topic_type = Bool
            elif topic_type == "sensor_msgs/msg/PointCloud2":
                topic_type = PointCloud2
            elif topic_type == "std_msgs/msg/Int16":
                topic_type = Int16
            elif topic_type == "std_msgs/msg/Float32":
                topic_type = Float32

            self.create_subscription(
                topic_type,
                topic_name,
                lambda msg, t=topic_name: self.topic_callback(msg, t),
                qos_profile_sensor_data, 
                raw=True
            )

    def topic_callback(self, msg, topic_name):
        now = self.get_clock().now()
        self.timestamps[topic_name].append(now)

    def get_active_nodes(self):
        discovered_nodes = self.get_node_names_and_namespaces()
        active_node_names = [name for (name, ns) in discovered_nodes]
        print(active_node_names)
        return active_node_names

    def check_hz_rate(self, topic, timeout_s=2.0):
        times = self.timestamps[topic]
        if len(times) < 2:
            return 0.0

        now = self.get_clock().now()
        last_time = times[-1]
        time_since_last = (now - last_time).nanoseconds * 1e-9

        if time_since_last > timeout_s:
            return 0.0

        total_time = (times[-1] - times[0]).nanoseconds * 1e-9
        if total_time <= 0.0:
            return 0.0
        n_msgs = len(times)
        hz = (n_msgs - 1) / total_time
        return hz

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
        self.node_leds = {}
        for node in self.node_list:
            self.node_leds[node] = StatusLed("")
            self.node_leds[node].setFixedWidth(20)
            self.node_leds[node].setDisabled(True)
            self.node_leds[node].set_state(0)
        self.topic_list = read_ros2_topic_list().get('ros2_topic_list', []).split(' ')
        self.topic_leds = {}
        for topic in self.topic_list:
            topic_name = topic.split(",")[0]
            self.topic_leds[topic_name] = StatusLed("")
            self.topic_leds[topic_name].setFixedWidth(20)
            self.topic_leds[topic_name].setDisabled(True)
            self.topic_leds[topic_name].set_state(0)
        self.controllers = read_robot_information().get('controllers', [])
        self.controllers_leds = {}
        for controller_type in self.controllers:
            self.controllers_leds[controller_type] = {}
            for controller in self.controllers[controller_type].split(' '):
                self.controllers_leds[controller_type][controller] = StatusLed("")
                self.controllers_leds[controller_type][controller].setFixedWidth(20)
                self.controllers_leds[controller_type][controller].setDisabled(True)
                self.controllers_leds[controller_type][controller].set_state(0)
        self.motors = read_robot_information().get('motors', [])
        self.motors_leds = {}
        for motor_type in self.motors:
            self.motors_leds[motor_type] = {}
            for motor in self.motors[motor_type].split(' '):
                self.motors_leds[motor_type][motor] = StatusLed("")
                self.motors_leds[motor_type][motor].setFixedWidth(20)
                self.motors_leds[motor_type][motor].setDisabled(True)
                self.motors_leds[motor_type][motor].set_state(0)

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

    def update_nodes(self):
        active_nodes = self.node.get_active_nodes()
        for i in reversed(range(self.nodes_content_layout.count())):
            self.nodes_content_layout.itemAt(i).widget().deleteLater()
        nodes_grid_layout = QGridLayout()
        for index, node in enumerate(self.node_list):
            self.node_leds[node].repaint_signal.emit(1 if node in active_nodes else 0)
            node_label = QLabel(node)
            node_label.setAlignment(Qt.AlignLeft)
            nodes_grid_layout.addWidget(self.node_leds[node], index, 0)
            nodes_grid_layout.addWidget(node_label, index, 1)
        nodes_grid_widget = QWidget()
        nodes_grid_widget.setLayout(nodes_grid_layout)
        self.nodes_content_layout.addWidget(nodes_grid_widget)

    def update_topics(self):
        for i in reversed(range(self.topics_content_layout.count())):
            self.topics_content_layout.itemAt(i).widget().deleteLater()
        topics_grid_layout = QGridLayout()

        for index, topic in enumerate(self.topic_list):
            topic = topic.split(",")[0]
            hz = self.node.check_hz_rate(topic)
            self.topic_leds[topic].repaint_signal.emit(1 if hz > 0 else 0)
            topic_label = QLabel(topic)
            topic_label.setAlignment(Qt.AlignLeft)
            topic_hz_label = QLabel(f"{hz:.2f} Hz")
            topic_hz_label.setFixedWidth(60)
            topic_hz_label.setAlignment(Qt.AlignRight)
            topics_grid_layout.addWidget(self.topic_leds[topic], index, 0)
            topics_grid_layout.addWidget(topic_label, index, 1)
            topics_grid_layout.addWidget(topic_hz_label, index, 2)

        topics_grid_widget = QWidget()
        topics_grid_widget.setLayout(topics_grid_layout)
        self.topics_content_layout.addWidget(topics_grid_widget)

    def update_controllers(self):
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
                self.controllers_leds[controller_type][controller].repaint_signal.emit(1 if controller in active_controllers else 0)
                controller_label = QLabel(controller)
                controller_label.setAlignment(Qt.AlignTop)
                controller_label.setAlignment(Qt.AlignLeft)
                controller_grid_layout.addWidget(self.controllers_leds[controller_type][controller], row_index, 0)
                controller_grid_layout.addWidget(controller_label, row_index, 1)
                row_index += 1

            controller_grid_widget = QWidget()
            controller_grid_widget.setLayout(controller_grid_layout)

            self.controllers_content.addWidget(controller_grid_widget)

    def update_motors(self):
        active_motors = self.node.get_active_motors()
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
                self.motors_leds[motor_type][motor].repaint_signal.emit(1 if motor in active_motors else 0)
                motor_label = QLabel(motor)
                motor_label.setAlignment(Qt.AlignLeft)
                motor_grid_layout.addWidget(self.motors_leds[motor_type][motor], row_index, 0)
                motor_grid_layout.addWidget(motor_label, row_index, 1)
                row_index += 1

            motor_grid_widget = QWidget()
            motor_grid_widget.setLayout(motor_grid_layout)

            self.motors_content.addWidget(motor_grid_widget)

    def update_columns(self):
        # Update nodes
        self.update_nodes()
        self.update_topics()
        self.update_controllers()
        self.update_motors()


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
