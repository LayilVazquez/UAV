"""
window.py
Main window: one tab per simulator, chosen at the top of the window, and a shared status bar.
"""
from PySide6.QtWidgets import QMainWindow, QTabWidget

from lib.gui.flocking_tab import FlockingTab
from lib.gui.formation_tab import FormationTab
from lib.gui.uav_tab import UavTab


class MainWindow(QMainWindow):
    """Holds the tabs. Each tab keeps its own fields, run and figures; switching tabs does not stop a run."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quadrotor simulator")
        self.resize(1400, 800)
        self.tabs = QTabWidget()
        self.tabs.addTab(UavTab(), "UAV simulator")
        self.tabs.addTab(FlockingTab(), "UAV flocking")
        self.tabs.addTab(FormationTab(), "Formation control")
        # The status bar is shared: show the status of the tab that comes on screen
        self.tabs.currentChanged.connect(
            lambda index: self.statusBar().showMessage(self.tabs.widget(index).last_status))
        self.setCentralWidget(self.tabs)
        self.statusBar().showMessage("Ready")
