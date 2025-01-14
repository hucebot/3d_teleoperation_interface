import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget
from PySide6.QtCore import Qt

class VideoWindow(QMainWindow):
    def __init__(self):
        super().__init__()
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoWindow()
    window.show()
    sys.exit(app.exec())