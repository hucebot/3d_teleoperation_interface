import sys
from PyQt5.QtWidgets import QApplication, QLabel, QGridLayout, QVBoxLayout, QHBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QObject
import dark_style
from threading import Thread

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


class GStreamerPipeline:
    def __init__(self, pipeline_str, callback):
        Gst.init(None)
        self.pipeline_str = pipeline_str
        self.callback = callback
        self.pipeline = Gst.parse_launch(self.pipeline_str)

        self.appsink = self.pipeline.get_by_name("appsink")
        self.appsink.connect("new-sample", self.on_new_sample)

    def start(self):
        self.loop = GLib.MainLoop()
        self.pipeline.set_state(Gst.State.PLAYING)
        self.thread = Thread(target=self.loop.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buffer = sample.get_buffer()
        caps = sample.get_caps()

        width = caps.get_structure(0).get_value("width")
        height = caps.get_structure(0).get_value("height")
        stride = 4 * width  # Assuming BGRx format

        success, map_info = buffer.map(Gst.MapFlags.READ)
        if success:
            image_data = map_info.data

            # Move processing to a separate thread
            Thread(target=self.callback, args=(width, height, stride, image_data), daemon=True).start()

            buffer.unmap(map_info)

        return Gst.FlowReturn.OK


class WebcamDisplay(QObject):
    image_updated = pyqtSignal(QImage)
    def __init__(self, label):
        super().__init__()
        self.label = label
        self.image_updated.connect(self._update_image)

    def update_image(self, width, height, stride, image_data):
        image = QImage(image_data, width, height, stride, QImage.Format_RGB32)
        self.image_updated.emit(image)

    def _update_image(self, image):
        pixmap = QPixmap.fromImage(image)
        self.label.setPixmap(pixmap)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teleoperation Interface")
        self.setGeometry(3000, 100, 1100, 800)

        main_layout = QGridLayout()

        # Upper area with multiple blocks
        self.upper_area = QWidget(self)
        self.upper_area.setFixedHeight(220)

        upper_layout = QHBoxLayout(self.upper_area)
        upper_layout.setContentsMargins(0, 0, 0, 0)
        upper_layout.setSpacing(10)

        # FPS GRAPH
        self.wifi_label = QLabel(self)
        self.wifi_label.setStyleSheet("background-color: #262629; font-size: 14px; padding: 10px;")
        upper_layout.addWidget(self.wifi_label)

        # DELAY GRAPH
        self.battery_label = QLabel(self)
        self.battery_label.setStyleSheet("background-color: #262629; font-size: 14px; padding: 10px;")
        upper_layout.addWidget(self.battery_label)

        # DOTS WITH CONNECTION WITH PC
        self.delay_label = QLabel(self)
        self.delay_label.setStyleSheet("background-color: #262629; font-size: 14px; padding: 10px;")
        upper_layout.addWidget(self.delay_label)

        # DATA
        self.time_label = QLabel(self)
        self.time_label.setStyleSheet("background-color: #262629; font-size: 14px; padding: 10px;")
        upper_layout.addWidget(self.time_label)

        main_layout.addWidget(self.upper_area, 0, 0, 1, 3)

        # Main camere area
        self.main_camera_area = QLabel(self)
        self.main_camera_area.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.main_camera_area, 1, 0, 1, 3)

        # Right camera
        right_layout = QVBoxLayout()
        self.right_camera_area = QLabel(self)
        self.right_camera_area.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.right_camera_area)

        # Left camera
        self.left_camera_area = QLabel(self)
        self.left_camera_area.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.left_camera_area)

        main_layout.addLayout(right_layout, 0, 3, 2, 1)

        self.setLayout(main_layout)

        # Setup GStreamer
        main_camera_pipeline = (
            "udpsrc port=5001 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! "
            "videoconvert ! videoscale ! video/x-raw,format=BGRx,width=1510,height=1150 ! appsink name=appsink emit-signals=true async=false"
        )
        self.webcam_display = WebcamDisplay(self.main_camera_area)
        self.gst_main_pipeline = GStreamerPipeline(main_camera_pipeline, self.webcam_display.update_image)
        self.gst_main_pipeline.start()

        left_camera_pipeline = (
            "udpsrc port=5000 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! "
            "videoconvert ! videoscale ! video/x-raw,format=BGRx,width=1000,height=700 ! appsink name=appsink emit-signals=true async=false"
        )
        self.webcam_display = WebcamDisplay(self.left_camera_area)
        self.gst_left_pipeline = GStreamerPipeline(left_camera_pipeline, self.webcam_display.update_image)
        self.gst_left_pipeline.start()

        right_camera_pipeline = (
            "udpsrc port=5002 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! "
            "videoconvert ! videoscale ! video/x-raw,format=BGRx,width=1000,height=700 ! appsink name=appsink emit-signals=true async=false"
        )
        self.webcam_display = WebcamDisplay(self.right_camera_area)
        self.gst_right_pipeline = GStreamerPipeline(right_camera_pipeline, self.webcam_display.update_image)
        self.gst_right_pipeline.start()


def main():
    app = QApplication(sys.argv)
    dark_style.dark_style(app)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()