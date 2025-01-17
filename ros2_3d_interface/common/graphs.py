from collections import deque
from PySide6.QtWidgets import QMainWindow
import pyqtgraph as pg


class WifiPlot(QMainWindow):
    def __init__(self, title, x_label, y_label, x_range, y_range, memory_limit=50):
        super().__init__()

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        self.title = title
        self.x_label = x_label
        self.y_label = y_label
        self.x_range = x_range
        self.y_range = y_range

        self.memory_limit = memory_limit
        self.x_values = deque(maxlen=self.memory_limit)
        self.y_values = deque(maxlen=self.memory_limit)

        self.graphWidget.setTitle(self.title, color="#ffffff", size="10pt")

        styles = {"color": "#ffffff", "font-size": "12px", "font-weight": "bold"}
        self.graphWidget.setLabel("left", self.y_label, **styles)
        #self.graphWidget.setLabel("bottom", self.x_label, **styles)

        self.graphWidget.addLegend()

        self.graphWidget.showGrid(x=False, y=True)
        self.graphWidget.setXRange(*self.x_range, padding=0)
        self.graphWidget.setYRange(*self.y_range, padding=0)

        self.curve = self.graphWidget.plot(pen=pg.mkPen(color=(0, 255, 15), width=2))

    def update_plot(self, x_value, y_value):
        self.x_values.append(x_value)
        self.y_values.append(y_value)

        self.curve.setData(list(self.x_values), list(self.y_values))

        if x_value > self.graphWidget.viewRange()[0][1] - 5:
            self.graphWidget.setXRange(x_value - self.memory_limit, x_value + 5, padding=0)

        if y_value < self.graphWidget.viewRange()[1][0] or y_value > self.graphWidget.viewRange()[1][1]:
            self.graphWidget.setYRange(
                min(self.y_values) - 5, max(self.y_values) + 5, padding=0
            )
