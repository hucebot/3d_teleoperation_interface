#!/usr/bin/env python

import sys
import threading

import rclpy
from rclpy.node import Node
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from ping3 import ping
from std_msgs.msg import String

from ros2_3d_interface.common.darkstyle import dark_style
from ros2_3d_interface.common.graphs import WifiPlot
from ros2_3d_interface.common.read_configuration import read_teleop_configuration

class SignalManager(QObject):
    status_signal = Signal(str)

class NetworkWindow(Node):
    def __init__(self, signal_manager, config_file):
        super().__init__('network_window_node')
        self.signal_manager = signal_manager
        plot_period = config_file["wifi_plots"]["plot_period"]
        self.timer = self.create_timer(plot_period, self.check_network_status)

        self.config = config_file

        self.robot_ip = self.config["general"]["robot_ip"]
        self.server = self.config["wifi_plots"]["server_name"]
        self.router = self.config["wifi_plots"]["router_ip"]

        self.get_logger().info("Robot Visualizer Node started!")

    def check_network_status(self):
        try:
            response_time = ping(self.robot_ip, timeout=2)
            if response_time is None:
                message = f"The robot {self.robot_ip} is not responding."
                self.get_logger().error(message)
            else:
                message = f"Robot {self.robot_ip}: Latency {response_time * 1000:.2f} ms"
            self.signal_manager.status_signal.emit(message)

            response_time = ping(self.server, timeout=2)
            if response_time is None:
                message = f"The server {self.server} is not responding."
                self.get_logger().error(message)
            else:
                message = f"Server {self.server}: Latency {response_time * 1000:.2f} ms"

            self.signal_manager.status_signal.emit(message)
            response_time = ping(self.router, timeout=2)
            if response_time is None:
                message = f"The router {self.router} is not responding."
                self.get_logger().error(message)
            else:
                message = f"Router {self.router}: Latency {response_time * 1000:.2f} ms"
            self.signal_manager.status_signal.emit(message)

        except Exception as e:
            error_message = f"Error checking network status: {e}"
            self.signal_manager.status_signal.emit(error_message)
            self.get_logger().error(error_message)


class NetworkWindowGUI(QMainWindow):
    def __init__(self, signal_manager, config_file):
        super().__init__()
        self.config_file = config_file
        self.setWindowTitle(self.config_file["wifi_plots"]["window_title"])
        x_position = self.config_file["wifi_plots"]["x_position"]
        y_position = self.config_file["wifi_plots"]["y_position"]
        width = self.config_file["wifi_plots"]["width"]
        height = self.config_file["wifi_plots"]["height"]
        self.setGeometry(x_position, y_position, width, height)

        self.plots = {}
        self.plots["Robot"] = WifiPlot(
            "Robot Ping", "Time", "Latency (ms)", 
            [self.config_file["wifi_plots"]["x_min"], self.config_file["wifi_plots"]["x_max"]], 
            [self.config_file["wifi_plots"]["y_min"], self.config_file["wifi_plots"]["y_max"]], 
            memory_limit=self.config_file["wifi_plots"]["memory_limit"]
        )

        self.plots["Server"] = WifiPlot(
            "Server Ping", "Time", "Latency (ms)", 
            [self.config_file["wifi_plots"]["x_min"], self.config_file["wifi_plots"]["x_max"]], 
            [self.config_file["wifi_plots"]["y_min"], self.config_file["wifi_plots"]["y_max"]], 
            memory_limit=self.config_file["wifi_plots"]["memory_limit"]
        )

        self.plots["Router"] = WifiPlot(
            "Router Ping", "Time", "Latency (ms)", 
            [self.config_file["wifi_plots"]["x_min"], self.config_file["wifi_plots"]["x_max"]], 
            [self.config_file["wifi_plots"]["y_min"], self.config_file["wifi_plots"]["y_max"]], 
            memory_limit=self.config_file["wifi_plots"]["memory_limit"]
        )


        plots_layout = QVBoxLayout()
        for plot in self.plots.values():
            plots_layout.addWidget(plot.graphWidget)

        self.bottom_label = QLabel("Network Status")
        self.bottom_label.setAlignment(Qt.AlignCenter)
        self.bottom_label.setStyleSheet("border: 1px solid black; background-color: lightgray;")

        main_layout = QVBoxLayout()
        main_layout.addLayout(plots_layout)
        main_layout.addWidget(self.bottom_label)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        signal_manager.status_signal.connect(self.handle_status_message)
        self.time_counter = 0

    def handle_status_message(self, message):
        self.bottom_label.setText(message)

        for server in self.plots.keys():
            if server in message and "Latency" in message:
                try:
                    latency = float(message.split("Latency ")[-1].replace(" ms", "").strip())
                    self.plots[server].update_plot(self.time_counter, latency)
                except ValueError:
                    pass

        self.time_counter += 1


def main(args=None):
    rclpy.init(args=args)
    signal_manager = SignalManager()
    config_file = read_teleop_configuration()

    node = NetworkWindow(signal_manager, config_file)

    app = QApplication(sys.argv)
    dark_style(app)

    gui = NetworkWindowGUI(signal_manager, config_file)
    gui.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app.exec()

    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
