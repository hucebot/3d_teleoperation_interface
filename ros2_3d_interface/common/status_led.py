from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QRadioButton
from PySide6.QtCore import Signal

class StatusLed(QRadioButton):
    repaint_signal = Signal(int)

    def __init__(self, text):
        super().__init__(text)
        self.state = 0
        self.setDisabled(True)
        self.repaint_signal.connect(self.update_state)  # Conectar la señal al método update_state
        self.set_state(self.state)

    def set_state(self, state):
        self.state = state
        if self.state == 0:
            self.set_color("#ff0000")
        elif self.state == 1:
            self.set_color("#0ffc00")
        self.update()

    def set_color(self, color):
        self.setStyleSheet(
            f"QRadioButton::indicator {{width: 14px; height: 14px; border-radius: 7px; background-color: {color};}}"
        )

    def update_state(self, state):
        self.set_state(state)
