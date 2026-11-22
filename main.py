"""
main.py
Starts the simulator window, with one tab per simulator (single UAV, UAV flocking, formation control).
The code lives in lib/; see the README of each subfolder.
"""
import sys

from PySide6.QtWidgets import QApplication

from lib.gui.style import apply_style
from lib.gui.window import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    apply_style(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
