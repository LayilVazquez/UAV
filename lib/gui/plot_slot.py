"""
plot_slot.py
One of the places of a tab where a figure is shown.
"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout
# After PySide6, so that Matplotlib binds to it and not to another installed Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT


class PlotSlot(QFrame):
    """Shows an interactive figure with its toolbar, or its title while empty."""

    def __init__(self, title):
        """title: shown while the slot is empty, and as its accessible name."""
        super().__init__()
        self.title = title
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumSize(240, 180)
        self.setAccessibleName(title)
        self.box = QVBoxLayout(self)
        self.clear()

    def empty_box(self):
        """Remove the label or the figure with its toolbar."""
        while self.box.count():
            self.box.takeAt(0).widget().deleteLater()

    def clear(self):
        """Back to the empty state: only the title."""
        self.empty_box()
        self.box.addWidget(QLabel(self.title, alignment=Qt.AlignCenter))

    def show_figure(self, figure):
        """Show a Matplotlib figure with its navigation toolbar (pan, zoom, save...)."""
        self.empty_box()
        figure.set_dpi(70)   # the figures are designed for 8 in; keeps the text readable in a small slot
        canvas = FigureCanvasQTAgg(figure)
        canvas.mpl_connect("resize_event", lambda event: figure.tight_layout())
        self.box.addWidget(NavigationToolbar2QT(canvas, self))
        self.box.addWidget(canvas, stretch=1)
