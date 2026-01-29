"""Advanced configuration tab for MKS servo motors."""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QLabel,
    QMessageBox,
    QVBoxLayout,
    QWidget,
)

from ...ui.theme import set_role
from ...ui.widgets import action_button
from ...ui.widgets import connect as qt_connect


_CHECKED = 2


class AdvancedConfigTab(QWidget):
    """Advanced motor configuration.

    - Limit switch remapping
    - Factory reset
    """

    operation_requested = pyqtSignal(str, dict)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(10, 10, 10, 10)

        layout.addWidget(self._build_limit_group())
        layout.addWidget(self._build_restore_group())
        layout.addStretch(1)

    def _build_limit_group(self) -> QGroupBox:
        group = QGroupBox("Limit Switch Configuration")
        v = QVBoxLayout(group)
        v.setSpacing(10)

        info = QLabel(
            "Enable limit switch remapping to use external limit switches "
            "instead of built-in homing switches."
        )
        info.setWordWrap(True)
        set_role(info, "muted")
        v.addWidget(info)

        self.limit_remap_check = QCheckBox("Enable Limit Switch Remapping")
        qt_connect(self.limit_remap_check.stateChanged, self._on_limit_remap_changed)
        v.addWidget(self.limit_remap_check)

        return group

    def _build_restore_group(self) -> QGroupBox:
        group = QGroupBox("Factory Reset")
        v = QVBoxLayout(group)
        v.setSpacing(10)

        warning = QLabel("This will restore all motor parameters to factory defaults.")
        set_role(warning, "danger")
        warning.setWordWrap(True)
        v.addWidget(warning)

        info = QLabel(
            "All custom settings including work mode, currents, subdivisions, and homing "
            "parameters will be lost."
        )
        info.setWordWrap(True)
        set_role(info, "muted")
        v.addWidget(info)

        restore_btn = action_button("Restore Factory Defaults", variant="danger")
        qt_connect(restore_btn.clicked, self._on_restore_defaults)
        v.addWidget(restore_btn)

        return group

    def _on_limit_remap_changed(self, state: int) -> None:
        self.operation_requested.emit("set_limit_remap", {"enable": state == _CHECKED})

    def _on_restore_defaults(self) -> None:
        reply = QMessageBox.warning(
            self,
            "Confirm Factory Reset",
            "Are you sure you want to restore factory defaults?\n\n"
            "All custom settings will be lost!",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.operation_requested.emit("restore_defaults", {})
