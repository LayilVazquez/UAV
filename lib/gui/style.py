"""
style.py
Dark theme with a glossy, crystal look in the spirit of Windows XP.
"""
from PySide6.QtGui import QColor, QPalette

BACKGROUND = "#101a2c"
PANEL = "#1b2a45"
FIELD = "#0b1322"
TEXT = "#e8eef7"
MUTED = "#9fb3cf"
BORDER = "#4f6f9f"
BLUE = "#245fc4"


def glass(base):
    """Crystal gradient: the hard break at the middle reads as a reflection."""
    color = QColor(base)
    return ("qlineargradient(x1:0, y1:0, x2:0, y2:1, "
            f"stop:0 {color.lighter(175).name()}, stop:0.48 {color.lighter(125).name()}, "
            f"stop:0.5 {color.name()}, stop:1 {color.lighter(140).name()})")


def button_rules(name, base, text):
    color = QColor(base)
    return f"""
QPushButton#{name} {{ background: {glass(base)}; color: {text}; border: 1px solid {color.darker(170).name()}; }}
QPushButton#{name}:hover {{ background: {glass(color.lighter(115).name())}; }}
QPushButton#{name}:pressed {{ background: {glass(color.darker(125).name())}; }}
QPushButton#{name}:disabled {{ background: {glass(color.darker(300).name())}; color: {MUTED};
                               border-color: {color.darker(300).name()}; }}
"""


STYLE = f"""
* {{ font-family: Tahoma, "Segoe UI"; font-size: 9pt; color: {TEXT}; }}
QMainWindow, QDialog {{ background: {BACKGROUND}; }}
QScrollArea, QScrollArea > QWidget > QWidget {{ background: transparent; border: none; }}
QLabel {{ background: transparent; }}

QGroupBox {{ background: {glass(PANEL)}; border: 1px solid {BORDER}; border-radius: 8px;
             margin-top: 14px; padding-top: 8px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 2px 10px; border-radius: 6px;
                    background: {glass(BLUE)}; font-weight: bold; }}
PlotSlot {{ background: {glass(PANEL)}; border: 1px solid {BORDER}; border-radius: 8px; }}

QLineEdit {{ border: 1px solid {BORDER}; border-radius: 5px; padding: 3px 6px;
             background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 {FIELD}, stop:1 {PANEL});
             selection-background-color: {BLUE}; }}
QLineEdit:focus {{ border: 1px solid #7fc4ff; }}

QPushButton {{ background: {glass(BLUE)}; border: 1px solid {BORDER}; border-radius: 7px;
               padding: 7px 10px; font-weight: bold; }}
QPushButton:hover {{ background: {glass(QColor(BLUE).lighter(115).name())}; }}
{button_rules("run", "#16861a", "#ffffff")}
{button_rules("save", "#e0ad00", "#1c1500")}
{button_rules("stop", "#c01a1a", "#ffffff")}

QToolBar {{ background: transparent; border: none; }}
QToolButton {{ background: transparent; border: 1px solid transparent; border-radius: 4px; }}
QToolButton:hover, QToolButton:checked {{ background: {glass(BLUE)}; border-color: {BORDER}; }}
QToolTip {{ background: {PANEL}; border: 1px solid {BORDER}; }}

QScrollBar:vertical {{ background: {FIELD}; width: 13px; }}
QScrollBar:horizontal {{ background: {FIELD}; height: 13px; }}
QScrollBar::handle {{ border: 1px solid {BORDER}; border-radius: 5px; }}
QScrollBar::handle:vertical {{ background: {glass(BLUE).replace("x2:0, y2:1", "x2:1, y2:0")}; min-height: 30px; }}
QScrollBar::handle:horizontal {{ background: {glass(BLUE)}; min-width: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line, QScrollBar::add-page, QScrollBar::sub-page {{ background: none; border: none; width: 0; height: 0; }}

QStatusBar {{ background: {BACKGROUND}; color: {MUTED}; font-size: 8pt; }}
"""


def apply_style(app):
    app.setStyle("Fusion")
    # Set the palette too: the Matplotlib toolbar picks its icon colour from it
    palette = QPalette()
    for role, color in {QPalette.Window: BACKGROUND, QPalette.WindowText: TEXT, QPalette.Base: FIELD,
                        QPalette.AlternateBase: PANEL, QPalette.Text: TEXT, QPalette.Button: PANEL,
                        QPalette.ButtonText: TEXT, QPalette.ToolTipBase: PANEL,
                        QPalette.ToolTipText: TEXT, QPalette.Highlight: BLUE}.items():
        palette.setColor(role, QColor(color))
    app.setPalette(palette)
    app.setStyleSheet(STYLE)
