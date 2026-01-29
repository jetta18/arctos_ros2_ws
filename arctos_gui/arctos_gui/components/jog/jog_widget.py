"""Jog widget for the Arctos GUI.

UI styling is driven by the application theme (see `arctos_gui/ui/theme.py`).
This widget focuses on behavior and uses dynamic properties (role/variant)
to select styling.
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListView,
    QPushButton,
    QScrollArea,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ...ui.theme import set_role, set_status, set_variant
from ...ui.widgets import action_button
from ...ui.widgets import connect as qt_connect
from ...ui.widgets import double_spinbox
from ...ui.widgets import field_label
from .jog_client_protocol import JogClient


class JogWidget(QWidget):
    """Jog control widget.

    Uses a `JogClient` to keep the Qt layer independent from ROS specifics.
    """

    AXIS_LABELS = ["X", "Y", "Z", "A", "B", "C"]

    def __init__(self, client: JogClient, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._client = client
        self._connected = False

        self._build_ui()
        self._setup_status_timer()

        self._log_message("System", "Jog widget initialized")
        self._update_status()

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded  # type: ignore[attr-defined]
        )
        scroll_area.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded  # type: ignore[attr-defined]
        )

        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(12)

        scroll_layout.addWidget(self._build_status_group())
        scroll_layout.addWidget(self._build_jog_group())
        scroll_layout.addWidget(self._build_log_group())
        scroll_layout.addStretch()

        scroll_area.setWidget(scroll_widget)
        main_layout.addWidget(scroll_area)

    def _build_status_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        row = QHBoxLayout()
        row.setSpacing(10)

        self._status_indicator = QLabel("●")
        set_role(self._status_indicator, "statusDot")
        set_status(self._status_indicator, "disconnected")

        self._status_text = QLabel("Disconnected")
        set_role(self._status_text, "statusText")
        set_status(self._status_text, "disconnected")

        self._connect_button = QPushButton("Connect")
        qt_connect(self._connect_button.clicked, self._toggle_connection)
        set_variant(self._connect_button, "primary")

        row.addWidget(self._status_indicator)
        row.addWidget(self._status_text)
        row.addStretch(1)
        row.addWidget(self._connect_button)
        layout.addLayout(row)

        info_row = QHBoxLayout()
        info_row.setSpacing(10)

        self._endpoint_label = QLabel(
            "ROS 2: /arctos_controller/joint_trajectory  |  /joint_states"
        )
        set_role(self._endpoint_label, "muted")

        self._last_update = QLabel("Last update: -")
        set_role(self._last_update, "muted")

        info_row.addWidget(self._endpoint_label)
        info_row.addStretch(1)
        info_row.addWidget(self._last_update)
        layout.addLayout(info_row)

        return group

    def _build_jog_group(self) -> QGroupBox:
        group = QGroupBox("Jog Controls")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(10)

        axis_label = field_label("Axis")
        self._axis_combo = QComboBox()
        self._axis_combo.addItems(self.AXIS_LABELS)
        self._axis_combo.setView(QListView())

        delta_label = field_label("Delta [rad]")
        self._delta_spin = double_spinbox(
            decimals=4,
            min_value=-6.283,
            max_value=6.283,
            step=0.1,
            value=0.1,
        )

        vel_label = field_label("Velocity [rad/s]")
        self._velocity_spin = double_spinbox(
            decimals=3,
            min_value=0.01,
            max_value=10.0,
            step=0.1,
            value=0.5,
        )

        layout.addWidget(axis_label, 0, 0)
        layout.addWidget(self._axis_combo, 0, 1)
        layout.addWidget(delta_label, 1, 0)
        layout.addWidget(self._delta_spin, 1, 1)
        layout.addWidget(vel_label, 2, 0)
        layout.addWidget(self._velocity_spin, 2, 1)

        button_row = QHBoxLayout()
        button_row.setSpacing(12)

        self._btn_minus = action_button("← Jog Negative", variant="primary")
        qt_connect(self._btn_minus.clicked, self._on_jog_minus)
        self._btn_minus.setEnabled(False)

        self._btn_plus = action_button("Jog Positive →", variant="primary")
        qt_connect(self._btn_plus.clicked, self._on_jog_plus)
        self._btn_plus.setEnabled(False)

        button_row.addStretch(1)
        button_row.addWidget(self._btn_minus)
        button_row.addWidget(self._btn_plus)
        button_row.addStretch(1)

        layout.addLayout(button_row, 3, 0, 1, 2)
        return group

    def _build_log_group(self) -> QGroupBox:
        group = QGroupBox("System Log")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        controls = QHBoxLayout()
        controls.addStretch(1)

        self._log_clear = QPushButton("Clear")
        qt_connect(self._log_clear.clicked, self._clear_log)
        controls.addWidget(self._log_clear)
        layout.addLayout(controls)

        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumHeight(220)
        set_role(self._log_text, "log")
        layout.addWidget(self._log_text)

        return group

    def _setup_status_timer(self) -> None:
        self._status_timer = QTimer()
        qt_connect(self._status_timer.timeout, self._update_status)
        self._status_timer.start(1000)

    def _update_status(self) -> None:
        is_connected = bool(self._client and self._client.is_connected())
        if is_connected != self._connected:
            self._set_connection_ui(is_connected)
            if is_connected:
                self._log_message("Connection", "Connected")
            else:
                self._log_message("Connection", "Disconnected")

        if is_connected:
            self._last_update.setText(
                f"Last update: {datetime.now().strftime('%H:%M:%S')}"
            )
        elif self._last_update.text() != "Last update: -":
            self._last_update.setText("Last update: -")

    def _set_connection_ui(self, connected: bool) -> None:
        self._connected = connected

        if connected:
            self._status_text.setText("Connected")
            set_status(self._status_indicator, "connected")
            set_status(self._status_text, "connected")

            self._connect_button.setText("Disconnect")
            set_variant(self._connect_button, "danger")

            self._btn_minus.setEnabled(True)
            self._btn_plus.setEnabled(True)
            return

        self._status_text.setText("Disconnected")
        set_status(self._status_indicator, "disconnected")
        set_status(self._status_text, "disconnected")

        self._connect_button.setText("Connect")
        set_variant(self._connect_button, "primary")

        self._btn_minus.setEnabled(False)
        self._btn_plus.setEnabled(False)

    def _log_message(self, source: str, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log_text.append(f"[{timestamp}] {source}: {message}")

        scrollbar = self._log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _clear_log(self) -> None:
        self._log_text.clear()
        self._log_message("System", "Log cleared")

    def _current_axis_index(self) -> int:
        return int(self._axis_combo.currentIndex())

    def _current_delta_rad(self) -> float:
        return float(self._delta_spin.value())

    def _current_velocity_rad_s(self) -> float:
        return float(self._velocity_spin.value())

    def _on_jog_plus(self) -> None:
        self._send_jog(direction=+1.0)

    def _on_jog_minus(self) -> None:
        self._send_jog(direction=-1.0)

    def _send_jog(self, direction: float) -> None:
        if not self._connected:
            self._log_message("Error", "Not connected")
            return

        axis_index = self._current_axis_index()
        axis_name = self.AXIS_LABELS[axis_index]
        delta_rad = self._current_delta_rad() * direction
        velocity_rad_s = self._current_velocity_rad_s()

        self._log_message(
            f"Jog {axis_name}",
            f"delta={delta_rad:.4f} rad, vel={velocity_rad_s:.3f} rad/s",
        )

        try:
            self._client.send_jog(axis_index, delta_rad, velocity_rad_s)
            self._log_message(f"Jog {axis_name}", "Command sent")
        except Exception as exc:  # noqa: BLE001
            self._log_message("Error", f"Jog failed: {exc}")

    def _toggle_connection(self) -> None:
        if self._connected:
            self._log_message("Connection", "Disconnecting...")
            try:
                self._client.disconnect()
            finally:
                self._update_status()
            return

        self._log_message("Connection", "Connecting...")
        try:
            ok = self._client.connect()
        except Exception as exc:  # noqa: BLE001
            ok = False
            self._log_message("Error", f"Connect failed: {exc}")

        if not ok:
            self._log_message("Error", "Connect failed")

        self._update_status()
