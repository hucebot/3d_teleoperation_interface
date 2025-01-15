from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QColor, QPalette
from PySide6.QtCore import Qt


def dark_style(app):
    app.setStyle('fusion')
    dark_palette = QPalette()

    dark_palette.setColor(QPalette.Window, QColor(0, 0, 0))
    dark_palette.setColor(QPalette.WindowText, Qt.white)
    dark_palette.setColor(QPalette.Base, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ToolTipBase, Qt.white)
    dark_palette.setColor(QPalette.ToolTipText, Qt.white)
    dark_palette.setColor(QPalette.Text, Qt.white)
    dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(
        QPalette.Disabled, QPalette.Button, QColor(30, 30, 30))
    dark_palette.setColor(QPalette.ButtonText, Qt.white)
    dark_palette.setColor(
        QPalette.Disabled, QPalette.ButtonText, QColor(100, 100, 100))
    dark_palette.setColor(QPalette.BrightText, Qt.red)
    dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(
        QPalette.Disabled, QPalette.Highlight, QColor(100, 100, 100))

    dark_palette.setColor(QPalette.HighlightedText, Qt.black)

    app.setPalette(dark_palette)

    app.setStyleSheet(
        "QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }"
    )