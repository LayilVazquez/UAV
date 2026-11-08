"""
plot_slot.py
One of the 5 places of the window where a figure is shown.
"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout
# After PySide6, so that Matplotlib binds to it and not to another installed Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT


class PlotSlot(QFrame):
    """Shows an interactive figure with its toolbar, or its title while empty."""

    def __init__(self, title):
        super().__init__()
        self.title = title
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumSize(240, 180)
        self.setAccessibleName(title)
        self.box = QVBoxLayout(self)
        self.clear()

    def empty_box(self):
        while self.box.count():
            self.box.takeAt(0).widget().deleteLater()

    def clear(self):
        self.empty_box()
        self.box.addWidget(QLabel(self.title, alignment=Qt.AlignCenter))

    def show_figure(self, figure):
        self.empty_box()
        figure.set_dpi(70)   # the figures are designed for 8 in; keeps the text readable in a small slot
        canvas = FigureCanvasQTAgg(figure)
        canvas.mpl_connect("resize_event", lambda event: figure.tight_layout())
        self.box.addWidget(NavigationToolbar2QT(canvas, self))
        self.box.addWidget(canvas, stretch=1)
