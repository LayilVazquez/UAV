"""
main.py
Starts the quadrotor simulator window. The code lives in lib/gui/.
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
