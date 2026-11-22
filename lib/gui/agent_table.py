"""
agent_table.py
AgentTable: one row per UAV (initial conditions, formation offsets...).
GraphTable: the N x N adjacency matrix of the communication graph.
Cells accept numbers only; a block copied from Excel or a text file can be pasted with Ctrl+V.
"""
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QKeySequence
from PySide6.QtWidgets import (QApplication, QHeaderView, QLineEdit, QStyledItemDelegate, QTableWidget,
                               QTableWidgetItem)

from lib.gui.fields import ANY
from lib.gui.simulator_tab import format_number, number_validator

COLUMNS = ["x", "y", "z", "vx", "vy", "vz"]   # default columns: initial position and velocity


class NumberDelegate(QStyledItemDelegate):
    """Edits the cells with the same number validator as the text fields."""

    def createEditor(self, parent, option, index):
        editor = QLineEdit(parent)
        editor.setValidator(number_validator(ANY))
        return editor


class AgentTable(QTableWidget):
    """One row per UAV: row i is UAV i + 1, one column per value (by default x, y, z [m], vx, vy, vz [m/s])."""

    def __init__(self, default_row, columns=COLUMNS):
        """default_row(i) gives the values proposed for a new row i (0-based), one per column."""
        super().__init__(0, len(columns))
        self.default_row = default_row
        self.setHorizontalHeaderLabels(columns)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.horizontalHeader().setMinimumSectionSize(40)   # the columns must fit the parameter panel
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

    def set_values(self, data, first_column=0):
        """Overwrite the cells of every row from first_column on with data (rows x k)."""
        for i, row in enumerate(data):
            for j, value in enumerate(row):
                self.setItem(i, first_column + j, QTableWidgetItem(format_number(value)))

    def values(self):
        """Return (rows x columns array, [names of invalid cells])."""
        data = np.zeros((self.rowCount(), self.columnCount()))
        invalid = []
        for i in range(self.rowCount()):
            for j in range(self.columnCount()):
                item = self.item(i, j)
                try:
                    data[i, j] = float(item.text() if item else "")
                    if not np.isfinite(data[i, j]):
                        raise ValueError
                except ValueError:
                    invalid.append(f"{self.verticalHeaderItem(i).text()}: {self.horizontalHeaderItem(j).text()}")
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


class GraphTable(AgentTable):
    """Adjacency matrix: a_ij > 0 means UAV i uses the data of UAV j, with weight a_ij.
    The diagonal is not used and cannot be edited."""

    def __init__(self):
        super().__init__(None, columns=[])
        self.setAccessibleName("Communication graph of the UAVs")

    def set_count(self, n):
        """Show an n x n matrix. Cells already there keep their value; new links default to 1."""
        old = self.rowCount()
        self.setRowCount(n)
        self.setColumnCount(n)
        for i in range(n):
            for j in range(n):
                if i >= old or j >= old:
                    item = QTableWidgetItem("0" if i == j else "1")
                    if i == j:
                        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                    self.setItem(i, j, item)
        labels = [f"UAV {i + 1}" for i in range(n)]
        self.setVerticalHeaderLabels(labels)
        self.setHorizontalHeaderLabels(labels)
