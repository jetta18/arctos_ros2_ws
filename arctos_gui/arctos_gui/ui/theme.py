"""Application theme and small styling helpers.

Qt widgets in this repo should avoid large per-widget stylesheets. Prefer:

- Application-wide QSS (uniform look)
- Per-widget dynamic properties (variants/roles)

This keeps UI code readable and makes the look consistent.
"""

from __future__ import annotations

from typing import Optional

from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QWidget


# Color system (light, professional)
_BG_TOP = "#f8fafc"
_BG_BOTTOM = "#eef2f7"
_SURFACE = "#ffffff"
_SURFACE_2 = "#f3f4f6"
_BORDER = "#d1d5db"
_TEXT = "#111827"
_TEXT_MUTED = "#6b7280"

_ACCENT = "#2563eb"
_ACCENT_HOVER = "#1d4ed8"
_ACCENT_PRESSED = "#1e40af"

_SUCCESS = "#16a34a"
_WARNING = "#f59e0b"
_DANGER = "#dc2626"

_MONO_FAMILY = '"Ubuntu Mono", "DejaVu Sans Mono"'


def apply_app_theme(app: QApplication) -> None:
    """Apply the global application theme."""
    # Fusion gives consistent metrics across desktop themes.
    app.setStyle("Fusion")

    font = QFont("Ubuntu", 10)
    if not font.exactMatch():
        font = QFont("Noto Sans", 10)
    app.setFont(font)

    app.setStyleSheet(_build_qss())


def set_variant(widget: QWidget, variant: Optional[str]) -> None:
    """Set a style variant (e.g. primary/secondary/danger) on a widget."""
    widget.setProperty("variant", variant)
    _repolish(widget)


def set_role(widget: QWidget, role: Optional[str]) -> None:
    """Set a style role (e.g. title/muted/log) on a widget."""
    widget.setProperty("role", role)
    _repolish(widget)


def set_status(widget: QWidget, status: Optional[str]) -> None:
    """Set a status (e.g. connected/disconnected) on a widget."""
    widget.setProperty("status", status)
    _repolish(widget)


def _repolish(widget: QWidget) -> None:
    # Dynamic property selectors require a polish to update.
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


def _build_qss() -> str:
    # Keep the QSS in one place so UI components don't carry styling noise.
    return f"""
/* Base */
QMainWindow {{
  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 {_BG_TOP}, stop:1 {_BG_BOTTOM});
}}

QWidget {{
  color: {_TEXT};
  font-size: 13px;
}}

QLabel[role="fieldLabel"] {{
  color: #374151;
  font-weight: 600;
}}

QPushButton[role="action"] {{
  min-height: 38px;
}}

QLabel[role="title"] {{
  font-size: 18px;
  font-weight: 700;
}}

QLabel[role="muted"] {{
  color: {_TEXT_MUTED};
}}

QLabel[role="accent"] {{
  color: {_ACCENT};
  font-weight: 600;
}}

QLabel[role="danger"] {{
  color: {_DANGER};
  font-weight: 700;
}}

QLabel[role="warning"] {{
  color: #b45309;
  font-weight: 700;
}}

QLabel[role="statusDot"] {{
  font-size: 18px;
  padding: 0 6px;
}}

QLabel[role="statusDot"][status="connected"] {{ color: {_SUCCESS}; }}
QLabel[role="statusDot"][status="disconnected"] {{ color: {_DANGER}; }}

QLabel[role="statusText"] {{
  font-weight: 700;
  font-size: 13px;
}}

QLabel[role="statusText"][status="connected"] {{ color: {_SUCCESS}; }}
QLabel[role="statusText"][status="disconnected"] {{ color: {_DANGER}; }}

/* Containers */
QGroupBox {{
  background: {_SURFACE};
  border: 1px solid {_BORDER};
  border-radius: 10px;
  margin-top: 10px;
  padding: 10px;
}}

QGroupBox::title {{
  subcontrol-origin: margin;
  left: 12px;
  padding: 0 6px;
  font-weight: 700;
}}

QFrame[role="card"] {{
  background: {_SURFACE};
  border: 1px solid {_BORDER};
  border-radius: 10px;
}}

/* Tabs */
QTabWidget::pane {{
  border: 1px solid {_BORDER};
  border-radius: 10px;
  background: {_SURFACE};
}}

QTabWidget#mainTabs QTabBar::tab {{
  background: {_SURFACE_2};
  border: 1px solid {_BORDER};
  border-bottom: none;
  padding: 8px 14px;
  margin-right: 4px;
  border-top-left-radius: 10px;
  border-top-right-radius: 10px;
  min-width: 140px;
}}

QTabWidget#subTabs QTabBar::tab {{
  background: {_SURFACE_2};
  border: 1px solid {_BORDER};
  border-bottom: none;
  padding: 6px 12px;
  margin-right: 4px;
  border-top-left-radius: 10px;
  border-top-right-radius: 10px;
}}

QTabBar::tab:selected {{
  background: {_SURFACE};
  border-color: {_BORDER};
}}

QTabBar::tab:hover {{
  background: #e9eef7;
}}

/* Inputs */
QLineEdit, QTextEdit, QPlainTextEdit, QComboBox, QSpinBox, QDoubleSpinBox {{
  background: {_SURFACE};
  border: 1px solid {_BORDER};
  border-radius: 8px;
  padding: 6px 8px;
}}

QSpinBox, QDoubleSpinBox {{
  padding-right: 34px;
}}

QSpinBox::up-button, QDoubleSpinBox::up-button {{
  subcontrol-origin: border;
  subcontrol-position: top right;
  width: 28px;
  background: {_SURFACE_2};
  border-left: 1px solid {_BORDER};
  border-bottom: 1px solid {_BORDER};
  border-top-right-radius: 8px;
}}

QSpinBox::down-button, QDoubleSpinBox::down-button {{
  subcontrol-origin: border;
  subcontrol-position: bottom right;
  width: 28px;
  background: {_SURFACE_2};
  border-left: 1px solid {_BORDER};
  border-top: 1px solid {_BORDER};
  border-bottom-right-radius: 8px;
}}

QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {{
  background: #e9eef7;
}}

QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {{
  image: none;
  width: 0;
  height: 0;
  border-left: 5px solid transparent;
  border-right: 5px solid transparent;
  border-bottom: 6px solid {_TEXT_MUTED};
}}

QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {{
  image: none;
  width: 0;
  height: 0;
  border-left: 5px solid transparent;
  border-right: 5px solid transparent;
  border-top: 6px solid {_TEXT_MUTED};
}}

QSpinBox::up-arrow:disabled, QDoubleSpinBox::up-arrow:disabled,
QSpinBox::down-arrow:disabled, QDoubleSpinBox::down-arrow:disabled {{
  border-top-color: #9ca3af;
  border-bottom-color: #9ca3af;
}}

QTextEdit[role="output"], QPlainTextEdit[role="output"] {{
  background: #f9fafb;
}}

QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus, QComboBox:focus,
QSpinBox:focus, QDoubleSpinBox:focus {{
  border: 2px solid {_ACCENT};
}}

QComboBox::drop-down {{
  border: none;
  width: 26px;
}}

QComboBox QAbstractItemView {{
  border: 1px solid {_BORDER};
  selection-background-color: {_ACCENT};
  selection-color: white;
  background: {_SURFACE};
  padding: 4px;
}}

/* Buttons */
QPushButton {{
  background: {_SURFACE_2};
  color: {_TEXT};
  border: 1px solid {_BORDER};
  border-radius: 8px;
  padding: 8px 14px;
  font-weight: 600;
}}

QPushButton:hover {{
  background: #e9eef7;
}}

QPushButton:pressed {{
  background: #dde6f5;
}}

QPushButton:disabled {{
  background: #e5e7eb;
  color: #9ca3af;
  border-color: #e5e7eb;
}}

QPushButton[variant="primary"] {{
  background: {_ACCENT};
  color: white;
  border-color: {_ACCENT};
}}

QPushButton[variant="primary"]:hover {{ background: {_ACCENT_HOVER}; }}
QPushButton[variant="primary"]:pressed {{ background: {_ACCENT_PRESSED}; }}

QPushButton[variant="danger"] {{
  background: {_DANGER};
  color: white;
  border-color: {_DANGER};
}}

QPushButton[variant="danger"]:hover {{ background: #b91c1c; }}
QPushButton[variant="danger"]:pressed {{ background: #991b1b; }}

QPushButton[variant="success"] {{
  background: {_SUCCESS};
  color: white;
  border-color: {_SUCCESS};
}}

QPushButton[variant="success"]:hover {{ background: #15803d; }}
QPushButton[variant="success"]:pressed {{ background: #166534; }}

QPushButton[variant="warning"] {{
  background: {_WARNING};
  color: #111827;
  border-color: {_WARNING};
}}

QPushButton[variant="warning"]:hover {{ background: #d97706; }}
QPushButton[variant="warning"]:pressed {{ background: #b45309; }}

/* Logs */
QTextEdit[role="log"], QPlainTextEdit[role="log"] {{
  font-family: {_MONO_FAMILY};
  font-size: 11px;
  background: #0b1220;
  color: #e5e7eb;
  border: 1px solid #1f2937;
}}

QTextEdit[role="terminal"], QPlainTextEdit[role="terminal"] {{
  font-family: {_MONO_FAMILY};
  font-size: 11px;
  background: #0f172a;
  color: #e5e7eb;
  border: 1px solid #243047;
}}

/* Scroll areas */
QScrollArea {{
  border: none;
  background: transparent;
}}

QScrollBar:vertical {{
  background: transparent;
  width: 10px;
  margin: 0;
}}

QScrollBar::handle:vertical {{
  background: #c7cfdd;
  border-radius: 5px;
  min-height: 20px;
}}

QScrollBar::handle:vertical:hover {{
  background: #aeb9cc;
}}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
  height: 0;
}}

QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
  background: transparent;
}}
"""
