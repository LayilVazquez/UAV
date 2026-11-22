"""
agent_table.py
AgentTable: one row per UAV with its initial position and velocity.
Cells accept numbers only; a block copied from Excel or a text file can be pasted with Ctrl+V.
"""
import numpy as np
from PySide6.QtGui import QKeySequence
from PySide6.QtWidgets import (QApplication, QHeaderView, QLineEdit, QStyledItemDelegate, QTableWidget,
                               QTableWidgetItem)

from lib.gui.fields import ANY
from lib.gui.simulator_tab import format_number, number_validator

COLUMNS = ["x", "y", "z", "vx", "vy", "vz"]


class NumberDelegate(QStyledItemDelegate):
    """Edits the cells with the same number validator as the text fields."""

    def createEditor(self, parent, option, index):
        editor = QLineEdit(parent)
        editor.setValidator(number_validator(ANY))
        return editor


class AgentTable(QTableWidget):
    """Initial conditions of the flock: row i is UAV i + 1, columns x, y, z [m] and vx, vy, vz [m/s]."""

    def __init__(self, default_row):
        """default_row(i) gives the 6 values proposed for a new row i (0-based)."""
        super().__init__(0, len(COLUMNS))
        self.default_row = default_row
        self.setHorizontalHeaderLabels(COLUMNS)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.horizontalHeader().setMinimumSectionSize(40)   # 6 columns must fit the parameter panel
        self.setItemDelegate(NumberDelegate(self))
        self.setMinimumHeight(240)
        self.setAccessibleName("Initial conditions of the UAVs")

    def set_count(self, n):
        """Show n rows. Rows already there keep what the user typed; new rows get default values."""
        old = self.rowCount()
        self.setRowCount(n)
        for i in range(old, n):
            for j, value in enumerate(self.default_row(i)):
                self.setItem(i, j, QTableWidgetItem(format_number(value)))
        self.setVerticalHeaderLabels([f"UAV {i + 1}" for i in range(n)])

    def values(self):
        """Return (N x 6 array, [names of invalid cells])."""
        data = np.zeros((self.rowCount(), len(COLUMNS)))
        invalid = []
        for i in range(self.rowCount()):
            for j, name in enumerate(COLUMNS):
                item = self.item(i, j)
                try:
                    data[i, j] = float(item.text() if item else "")
                    if not np.isfinite(data[i, j]):
                        raise ValueError
                except ValueError:
                    invalid.append(f"UAV {i + 1}: {name}")
        return data, invalid

    def keyPressEvent(self, event):
        """Ctrl+V pastes a block of cells; other keys work as usual."""
        if event.matches(QKeySequence.Paste):
            self.paste()
        else:
            super().keyPressEvent(event)

    def paste(self):
        """Paste rows of numbers separated by tabs or spaces, starting at the current cell.
        Cells that fall outside the table are ignored: set the number of UAVs first."""
        top, left = max(self.currentRow(), 0), max(self.currentColumn(), 0)
        for i, line in enumerate(QApplication.clipboard().text().splitlines()):
            for j, text in enumerate(line.split()):
                if top + i < self.rowCount() and left + j < self.columnCount():
                    self.setItem(top + i, left + j, QTableWidgetItem(text))
