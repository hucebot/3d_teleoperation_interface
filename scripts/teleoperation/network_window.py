import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget
from PySide6.QtCore import Qt

class RobotVisualizer(QMainWindow):
    def __init__(self, pos_x, pos_y, width, height):
        super().__init__()
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