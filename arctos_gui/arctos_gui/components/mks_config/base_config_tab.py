"""Base configuration tab for MKS servo motors."""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGroupBox,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from ...ui.widgets import action_button
from ...ui.widgets import connect as qt_connect
from ...ui.widgets import field_label
from ...ui.widgets import int_spinbox


class BaseConfigTab(QWidget):
    """Basic motor configuration.

    Emits `operation_requested` with the operation name and parameters.
    The motor id is resolved by the parent widget.
    """

    operation_requested = pyqtSignal(str, dict)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(10, 10, 10, 10)

        layout.addWidget(self._build_work_mode_group())
        layout.addWidget(self._build_current_group())
        layout.addWidget(self._build_microstep_group())
        layout.addStretch(1)

    def _build_work_mode_group(self) -> QGroupBox:
        group = QGroupBox("Work Mode")
        form = QFormLayout(group)
        form.setSpacing(10)
        form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        self.work_mode_combo = QComboBox()
        self.work_mode_combo.addItems(
            [
                "0 - CR Open (Open Loop)",
                "1 - CR Close (Closed Loop)",
                "2 - CR_vFOC (Closed Loop FOC)",
                "3 - SR Open (Open Loop Speed)",
                "4 - SR Close (Closed Loop Speed)",
                "5 - SR_vFOC (Closed Loop Speed FOC)",
            ]
        )
        self.work_mode_combo.setCurrentIndex(5)
        form.addRow(field_label("Mode:"), self.work_mode_combo)

        set_mode_btn = action_button("Set Work Mode", variant="primary")
        qt_connect(set_mode_btn.clicked, self._on_set_work_mode)
        form.addRow(QLabel(""), set_mode_btn)

        return group

    def _build_current_group(self) -> QGroupBox:
        group = QGroupBox("Current Settings")
        form = QFormLayout(group)
        form.setSpacing(10)
        form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        self.working_current_spin = int_spinbox(
            min_value=100,
            max_value=3000,
            value=1600,
            suffix=" mA",
        )
        form.addRow(field_label("Working Current:"), self.working_current_spin)

        set_current_btn = action_button("Set Working Current", variant="primary")
        qt_connect(set_current_btn.clicked, self._on_set_working_current)
        form.addRow(QLabel(""), set_current_btn)

        self.holding_current_spin = int_spinbox(
            min_value=10,
            max_value=90,
            step=10,
            value=70,
            suffix=" %",
        )
        form.addRow(field_label("Holding Current:"), self.holding_current_spin)

        set_holding_btn = action_button("Set Holding Current", variant="primary")
        qt_connect(set_holding_btn.clicked, self._on_set_holding_current)
        form.addRow(QLabel(""), set_holding_btn)

        return group

    def _build_microstep_group(self) -> QGroupBox:
        group = QGroupBox("Microstep Subdivision")
        form = QFormLayout(group)
        form.setSpacing(10)
        form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        self.subdivision_combo = QComboBox()
        self.subdivision_combo.addItems(
            [
                "0 - Full Step",
                "1 - 2 Microsteps",
                "2 - 4 Microsteps",
                "3 - 8 Microsteps",
                "4 - 16 Microsteps",
                "5 - 32 Microsteps",
                "6 - 64 Microsteps",
                "7 - 128 Microsteps",
                "8 - 256 Microsteps",
            ]
        )
        self.subdivision_combo.setCurrentIndex(7)
        form.addRow(field_label("Subdivision:"), self.subdivision_combo)

        set_subdivision_btn = action_button("Set Subdivision", variant="primary")
        qt_connect(set_subdivision_btn.clicked, self._on_set_subdivision)
        form.addRow(QLabel(""), set_subdivision_btn)

        return group

    def _on_set_work_mode(self) -> None:
        self.operation_requested.emit(
            "set_work_mode",
            {"work_mode": self.work_mode_combo.currentIndex()},
        )

    def _on_set_working_current(self) -> None:
        self.operation_requested.emit(
            "set_working_current",
            {"current_ma": int(self.working_current_spin.value())},
        )

    def _on_set_holding_current(self) -> None:
        self.operation_requested.emit(
            "set_holding_current",
            {"percentage": int(self.holding_current_spin.value())},
        )

    def _on_set_subdivision(self) -> None:
        self.operation_requested.emit(
            "set_subdivision",
            {"subdivision": self.subdivision_combo.currentIndex()},
        )
