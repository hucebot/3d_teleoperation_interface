import sys
import os
import uuid
from PySide6.QtCore import QTimer, QUrl
from PySide6.QtGui import QGuiApplication, QVector3D
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DRender import Qt3DRender
from PySide6.Qt3DExtras import Qt3DExtras
import cv2


class MainWindow(Qt3DExtras.Qt3DWindow):
    def __init__(self):
        super().__init__()

        self.root_entity = Qt3DCore.QEntity()

        # Camera configuration
        self.camera_x_position, self.camera_y_position, self.camera_z_position = -3, 114, 13
        self.camera_pitch, self.camera_roll, self.camera_yaw = -2.4, 74.0, 8.74
        self.camera_x_up, self.camera_y_up, self.camera_z_up = 0.0, 0.0, -1.0
        self.camera().setPosition(QVector3D(self.camera_x_position, self.camera_y_position, self.camera_z_position))
        self.camera().setUpVector(QVector3D(self.camera_x_up, self.camera_y_up, self.camera_z_up))
        self.camera().setViewCenter(QVector3D(self.camera_pitch, self.camera_roll, self.camera_yaw))

        # Background configuration
        self.background_entity = Qt3DCore.QEntity(self.root_entity)
        self.background_mesh = Qt3DExtras.QPlaneMesh()
        self.background_mesh.setWidth(50)
        self.background_mesh.setHeight(50)

        # Initialize video stream
        self.video_capture = cv2.VideoCapture(1)  # Webcam index (1 for the user's webcam)
        if not self.video_capture.isOpened():
            raise RuntimeError("Webcam could not be opened!")

        # Temporary path template for frames
        self.temp_dir = "/tmp"
        self.texture_path_template = os.path.join(self.temp_dir, "webcam_frame_{}.jpg")

        # Create the texture
        self.texture_image = Qt3DRender.QTextureImage()
        self.background_texture = Qt3DRender.QTexture2D()
        self.background_texture.addTextureImage(self.texture_image)
        self.background_texture.setGenerateMipMaps(False)
        self.background_texture.setMagnificationFilter(Qt3DRender.QAbstractTexture.Linear)
        self.background_texture.setMinificationFilter(Qt3DRender.QAbstractTexture.Linear)

        # Create the material and assign the texture
        self.background_material = Qt3DExtras.QDiffuseMapMaterial()
        self.background_material.setDiffuse(self.background_texture)

        # Configure the background entity
        self.background_entity.addComponent(self.background_mesh)
        self.background_entity.addComponent(self.background_material)

        # Timer to update the video texture
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_video_texture)
        self.timer.start(30)  # Update approximately 30 times per second

        # Set the root entity
        self.setRootEntity(self.root_entity)

    def update_video_texture(self):
        # Capture frame from webcam
        ret, frame = self.video_capture.read()
        if not ret:
            print("Failed to capture video frame")
            return

        # Convert frame to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Save the frame as an image with a unique name
        unique_id = uuid.uuid4().hex
        texture_path = self.texture_path_template.format(unique_id)
        cv2.imwrite(texture_path, frame)

        # Update the texture source
        self.texture_image.setSource(QUrl.fromLocalFile(texture_path))

        # Clean up old textures
        self.cleanup_old_textures(unique_id)

    def cleanup_old_textures(self, current_id):
        """Removes old texture files except the latest one."""
        for filename in os.listdir(self.temp_dir):
            if filename.startswith("webcam_frame_") and not filename.endswith(f"{current_id}.jpg"):
                try:
                    os.remove(os.path.join(self.temp_dir, filename))
                except OSError:
                    pass

    def closeEvent(self, event):
        # Release the video capture on exit and delete all temporary files
        self.video_capture.release()
        for filename in os.listdir(self.temp_dir):
            if filename.startswith("webcam_frame_"):
                try:
                    os.remove(os.path.join(self.temp_dir, filename))
                except OSError:
                    pass
        super().closeEvent(event)


if __name__ == '__main__':
    app = QGuiApplication(sys.argv)
    view = MainWindow()
    view.show()
    sys.exit(app.exec())
