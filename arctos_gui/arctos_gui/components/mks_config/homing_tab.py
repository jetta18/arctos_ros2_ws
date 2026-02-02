"""Homing configuration tab for MKS servo motors."""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
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
from ...ui.widgets import int_spinbox


class HomingTab(QWidget):
    """Configure and run homing operations."""

    operation_requested = pyqtSignal(str, dict)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(10, 10, 10, 10)

        layout.addWidget(self._build_params_group())
        layout.addWidget(self._build_ops_group())

    def _build_params_group(self) -> QGroupBox:
        group = QGroupBox("Homing Parameters")
        form = QFormLayout(group)
        form.setSpacing(10)
        form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)

        self.home_trigger_combo = combo_box(items=["0 - Low Level", "1 - High Level"])
        form.addRow(field_label("Trigger Level:"), self.home_trigger_combo)

        self.home_direction_combo = combo_box(
            items=["0 - Clockwise", "1 - Counter-Clockwise"]
        )
        form.addRow(field_label("Direction:"), self.home_direction_combo)

        self.home_speed_spin = int_spinbox(
            min_value=1,
            max_value=1000,
            value=100,
            suffix=" RPM",
        )
        form.addRow(field_label("Homing Speed:"), self.home_speed_spin)

        self.home_enable_limit_check = QCheckBox("Enable Limit Switch")
        self.home_enable_limit_check.setChecked(True)
        form.addRow(QLabel(""), self.home_enable_limit_check)

        set_params_btn = action_button("Set Homing Parameters", variant="primary")
        qt_connect(set_params_btn.clicked, self._on_set_home_params)
        form.addRow(QLabel(""), set_params_btn)

        return group

    def _build_ops_group(self) -> QGroupBox:
        group = QGroupBox("Homing Operations")
        v = QVBoxLayout(group)
        v.setSpacing(10)

        self.home_btn = action_button("Start Homing", variant="primary")
        qt_connect(self.home_btn.clicked, self._on_go_home)

        self.set_zero_btn = action_button("Set Current Position as Zero", variant="warning")
        qt_connect(self.set_zero_btn.clicked, self._on_set_zero)

        v.addWidget(action_button_row(self.home_btn, self.set_zero_btn))

        info = QLabel(
            "• Start Homing: Moves the motor to the home/limit switch\n"
            "• Set Zero: Marks the current position as zero reference"
        )
        set_role(info, "muted")
        v.addWidget(info)

        return group

    def _on_set_home_params(self) -> None:
        params = {
            "trigger": self.home_trigger_combo.currentIndex(),
            "direction": self.home_direction_combo.currentIndex(),
            "speed": int(self.home_speed_spin.value()),
            "enable_limit": bool(self.home_enable_limit_check.isChecked()),
        }
        self.operation_requested.emit("set_home_params", params)

    def _on_go_home(self) -> None:
        reply = QMessageBox.question(
            self,
            "Confirm Homing",
            "Start the homing sequence?\nThe motor will move to find the home position.",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.operation_requested.emit("go_home", {})

    def _on_set_zero(self) -> None:
        reply = QMessageBox.question(
            self,
            "Confirm Zero Position",
            "Set the current position as zero reference?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.operation_requested.emit("set_zero", {})
