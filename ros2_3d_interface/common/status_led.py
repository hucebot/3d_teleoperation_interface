from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QRadioButton
from PySide6.QtCore import Signal

class StatusLed(QRadioButton):
    
    repaint_signal = Signal()

    def __init__(self, text):
        super().__init__(text)
        self.state = 0
        self.setDisabled(True)
        self.set_state(self.state)

    def set_state(self, state):
        self.state = state
        if self.state == 0:
            self.set_color("red")
        elif self.state == 1:
            self.set_color("green")
        else:
            self.set_color("orange")

        self.update()

    def set_color(self, color):
        self.setStyleSheet(
            "QRadioButton::indicator {width: 14px; height: 14px; border-radius: 7px;} QRadioButton::indicator:unchecked { background-color:" + color + "}") 