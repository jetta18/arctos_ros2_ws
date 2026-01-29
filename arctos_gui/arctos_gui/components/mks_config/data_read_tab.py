"""Data reading tab for MKS servo motors."""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGroupBox,
    QLabel,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ...ui.theme import set_role
from ...ui.widgets import action_button
from ...ui.widgets import action_button_row
from ...ui.widgets import connect as qt_connect
from ...ui.widgets import field_label


_MOTOR_OPTIONS = [
    "X - Joint 1",
    "Y - Joint 2",
    "Z - Joint 3",
    "A - Joint 4",
    "B - Joint 5",
    "C - Joint 6",
]

_ACCENT = "#2563eb"
_MUTED = "#6b7280"
_SUCCESS = "#16a34a"
_DANGER = "#dc2626"
_WARNING = "#f59e0b"


class DataReadTab(QWidget):
    """Read encoder, speed, IO, and motor status."""

    operation_requested = pyqtSignal(str, dict)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._init_ui()
        self._on_motor_changed(0)

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(10, 10, 10, 10)

        layout.addWidget(self._build_motor_selection_group())
        layout.addWidget(self._build_read_controls_group())
        layout.addWidget(self._build_display_group())
        layout.addStretch(1)

    def _build_motor_selection_group(self) -> QGroupBox:
        group = QGroupBox("Motor Selection")
        form = QFormLayout(group)
        form.setSpacing(10)
        form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)

        self.motor_combo = QComboBox()
        self.motor_combo.addItems(_MOTOR_OPTIONS)
        qt_connect(self.motor_combo.currentIndexChanged, self._on_motor_changed)
        form.addRow(field_label("Select Motor:"), self.motor_combo)

        self.motor_id_label = QLabel("Motor ID: 1")
        set_role(self.motor_id_label, "accent")
        form.addRow(QLabel(""), self.motor_id_label)

        return group

    def _build_read_controls_group(self) -> QGroupBox:
        group = QGroupBox("Data Read Operations")
        v = QVBoxLayout(group)
        v.setSpacing(10)

        encoder_btn = action_button("Read Encoder")
        qt_connect(encoder_btn.clicked, self._on_read_encoder)

        speed_btn = action_button("Read Speed")
        qt_connect(speed_btn.clicked, self._on_read_speed)

        v.addWidget(action_button_row(encoder_btn, speed_btn))

        io_btn = action_button("Read I/O Status")
        qt_connect(io_btn.clicked, self._on_read_io_status)

        status_btn = action_button("Read Motor Status")
        qt_connect(status_btn.clicked, self._on_read_motor_status)

        v.addWidget(action_button_row(io_btn, status_btn))

        read_all_btn = action_button("Read All Data", variant="primary")
        qt_connect(read_all_btn.clicked, self._on_read_all)
        v.addWidget(read_all_btn)

        return group

    def _build_display_group(self) -> QGroupBox:
        group = QGroupBox("Data Display")
        v = QVBoxLayout(group)
        v.setSpacing(6)

        v.addWidget(field_label("Encoder Position:"))
        self.encoder_display = QTextEdit()
        self.encoder_display.setReadOnly(True)
        self.encoder_display.setMaximumHeight(80)
        set_role(self.encoder_display, "output")
        v.addWidget(self.encoder_display)

        v.addWidget(field_label("Motor Speed:"))
        self.speed_display = QTextEdit()
        self.speed_display.setReadOnly(True)
        self.speed_display.setMaximumHeight(60)
        set_role(self.speed_display, "output")
        v.addWidget(self.speed_display)

        v.addWidget(field_label("I/O Status:"))
        self.io_display = QTextEdit()
        self.io_display.setReadOnly(True)
        self.io_display.setMaximumHeight(100)
        set_role(self.io_display, "output")
        v.addWidget(self.io_display)

        v.addWidget(field_label("Motor Status:"))
        self.motor_status_display = QTextEdit()
        self.motor_status_display.setReadOnly(True)
        self.motor_status_display.setMaximumHeight(120)
        set_role(self.motor_status_display, "output")
        v.addWidget(self.motor_status_display)

        self.clear_displays()
        return group

    def get_motor_id(self) -> int:
        return int(self.motor_combo.currentIndex()) + 1

    def clear_displays(self) -> None:
        muted = f"<span style='color: {_MUTED};'>No data</span>"
        self.encoder_display.setHtml(muted)
        self.speed_display.setHtml(muted)
        self.io_display.setHtml(muted)
        self.motor_status_display.setHtml(muted)

    def _on_motor_changed(self, index: int) -> None:
        motor_id = int(index) + 1
        self.motor_id_label.setText(f"Motor ID: {motor_id}")
        self.clear_displays()

    def _on_read_encoder(self) -> None:
        self.operation_requested.emit("read_encoder", {"motor_id": self.get_motor_id()})

    def _on_read_speed(self) -> None:
        self.operation_requested.emit("read_speed", {"motor_id": self.get_motor_id()})

    def _on_read_io_status(self) -> None:
        self.operation_requested.emit("read_io_status", {"motor_id": self.get_motor_id()})

    def _on_read_motor_status(self) -> None:
        self.operation_requested.emit(
            "read_motor_status",
            {"motor_id": self.get_motor_id()},
        )

    def _on_read_all(self) -> None:
        motor_id = self.get_motor_id()
        self.clear_displays()

        self.operation_requested.emit("read_encoder", {"motor_id": motor_id})
        self.operation_requested.emit("read_speed", {"motor_id": motor_id})
        self.operation_requested.emit("read_io_status", {"motor_id": motor_id})
        self.operation_requested.emit("read_motor_status", {"motor_id": motor_id})

    def update_encoder_display(self, data: dict) -> None:
        self.encoder_display.setHtml(
            f"""<b style='color: {_ACCENT};'>Encoder</b><br>
<b>Raw Value:</b> {data.get('encoder_raw_value', 0)}<br>
<b>Angle (deg):</b> {data.get('encoder_angle_degrees', 0):.2f}&deg;<br>
<b>Angle (rad):</b> {data.get('encoder_angle_radians', 0):.4f} rad"""
        )

    def update_speed_display(self, data: dict) -> None:
        self.speed_display.setHtml(
            f"""<b style='color: {_ACCENT};'>Speed</b><br>
<b>RPM:</b> {data.get('speed_rpm', 0)}<br>
<b>rad/s:</b> {data.get('speed_rad_per_sec', 0):.4f} rad/s"""
        )

    def update_io_display(self, data: dict) -> None:
        def _hl(value: bool) -> str:
            color = _SUCCESS if value else _DANGER
            return f"<span style='color: {color}; font-weight: 700;'>{'HIGH' if value else 'LOW'}</span>"

        stall = bool(data.get("stall_detected"))
        stall_color = _DANGER if stall else _SUCCESS
        stall_text = "YES" if stall else "NO"

        self.io_display.setHtml(
            f"""<b style='color: {_ACCENT};'>I/O</b><br>
<b>IN1 (Home/Left Limit):</b> {_hl(bool(data.get('io_in1')))}<br>
<b>IN2 (Right Limit):</b> {_hl(bool(data.get('io_in2')))}<br>
<b>OUT1 (Stall Detection):</b> {_hl(bool(data.get('io_out1')))}<br>
<b>OUT2:</b> {_hl(bool(data.get('io_out2')))}<br>
<b>Stall Detected:</b> <span style='color: {stall_color}; font-weight: 700;'>{stall_text}</span>"""
        )

    def update_motor_status_display(self, data: dict) -> None:
        def _yes_no(value: bool, true_color: str, false_color: str) -> str:
            return (
                f"<span style='color: {true_color if value else false_color}; font-weight: 700;'>"
                f"{'YES' if value else 'NO'}</span>"
            )

        enabled = bool(data.get("motor_enabled"))
        moving = bool(data.get("motor_moving"))
        calibrated = bool(data.get("motor_calibrated"))
        error = bool(data.get("motor_error"))

        self.motor_status_display.setHtml(
            f"""<b style='color: {_ACCENT};'>Motor Status</b><br>
<b>Enabled:</b> {_yes_no(enabled, _SUCCESS, _DANGER)}<br>
<b>Moving:</b> {_yes_no(moving, _WARNING, _SUCCESS)}<br>
<b>Calibrated:</b> {_yes_no(calibrated, _SUCCESS, _DANGER)}<br>
<b>Error:</b> {_yes_no(error, _DANGER, _SUCCESS)}<br>
<b>Status Code:</b> {data.get('motor_status_code', 0)}<br>
<b>Status:</b> {data.get('motor_status_text', 'Unknown')}"""
        )
