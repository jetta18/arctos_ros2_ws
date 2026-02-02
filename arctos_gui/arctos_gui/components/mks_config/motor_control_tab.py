"""Motor control tab for MKS servo motors."""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QLabel,
    QMessageBox,
    QVBoxLayout,
    QWidget,
)

from ...ui.theme import set_role
from ...ui.widgets import action_button
from ...ui.widgets import action_button_row
from ...ui.widgets import combo_box
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


class MotorControlTab(QWidget):
    """Enable/disable motors and query status."""

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
        layout.addWidget(self._build_control_group())

    def _build_motor_selection_group(self) -> QGroupBox:
        group = QGroupBox("Motor Selection")
        form = QFormLayout(group)
        form.setSpacing(10)
        form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)

        self.motor_combo = combo_box(items=_MOTOR_OPTIONS)
        qt_connect(self.motor_combo.currentIndexChanged, self._on_motor_changed)
        form.addRow(field_label("Select Motor:"), self.motor_combo)

        self.motor_id_label = QLabel("Motor ID: 1")
        set_role(self.motor_id_label, "accent")
        form.addRow(QLabel(""), self.motor_id_label)

        return group

    def _build_control_group(self) -> QGroupBox:
        group = QGroupBox("Motor Control")
        v = QVBoxLayout(group)
        v.setSpacing(10)

        self.enable_btn = action_button("Enable Motor", variant="success")
        qt_connect(self.enable_btn.clicked, self._on_enable_clicked)

        self.disable_btn = action_button("Disable Motor", variant="danger")
        qt_connect(self.disable_btn.clicked, self._on_disable_clicked)

        v.addWidget(action_button_row(self.enable_btn, self.disable_btn))

        status_btn = action_button("Query Motor Status")
        qt_connect(status_btn.clicked, self._on_query_status)
        v.addWidget(status_btn)

        info = QLabel(
            "• Enable/Disable: Turn motor power on or off\n"
            "• Query Status: Read current motor status information"
        )
        set_role(info, "muted")
        v.addWidget(info)

        return group

    def get_motor_id(self) -> int:
        return int(self.motor_combo.currentIndex()) + 1

    def _on_motor_changed(self, index: int) -> None:
        motor_id = int(index) + 1
        self.motor_id_label.setText(f"Motor ID: {motor_id}")

    def _on_enable_clicked(self) -> None:
        self._request_enable(True)

    def _on_disable_clicked(self) -> None:
        self._request_enable(False)

    def _request_enable(self, enable: bool) -> None:
        motor_id = self.get_motor_id()

        if not enable:
            reply = QMessageBox.question(
                self,
                "Confirm Disable",
                f"Disable motor {motor_id}?\nThe motor will lose power.",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return

        self.operation_requested.emit(
            "enable_motor",
            {"motor_id": motor_id, "enable": enable},
        )

    def _on_query_status(self) -> None:
        self.operation_requested.emit(
            "query_status",
            {"motor_id": self.get_motor_id()},
        )
