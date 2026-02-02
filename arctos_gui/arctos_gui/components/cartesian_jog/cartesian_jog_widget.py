"""Cartesian jog control widget for moving the robot in X/Y/Z space."""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ...ui.theme import set_role, set_status, set_variant
from ...ui.widgets import action_button
from ...ui.widgets import connect as qt_connect
from ...ui.widgets import double_spinbox
from ...ui.widgets import field_label
from .cartesian_jog_client_protocol import CartesianJogClient


class CartesianJogWidget(QWidget):
    """Cartesian jog control widget.

    Uses a `CartesianJogClient` to keep the Qt layer independent from MoveIt specifics.
    """

    AXES = ["X", "Y", "Z", "RX", "RY", "RZ"]

    def __init__(self, client: CartesianJogClient, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._client = client
        self._connected = False

        self._build_ui()
        self._setup_status_timer()
        self._setup_pose_timer()

        self._log_message("System", "Cartesian jog widget initialized")
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
        scroll_layout.addWidget(self._build_controls_group())
        scroll_layout.addWidget(self._build_pose_group())
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

        self._endpoint_label = QLabel("MoveIt: arctos_arm → Link_6_1 (World Frame, Pilz LIN)")
        set_role(self._endpoint_label, "muted")

        self._last_update = QLabel("Last update: -")
        set_role(self._last_update, "muted")

        info_row.addWidget(self._endpoint_label)
        info_row.addStretch(1)
        info_row.addWidget(self._last_update)
        layout.addLayout(info_row)

        return group

    def _build_controls_group(self) -> QGroupBox:
        group = QGroupBox("Jog Parameters")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(10)

        step_label = field_label("Step Size [mm]")
        self._step_spin = double_spinbox(
            decimals=1,
            min_value=0.1,
            max_value=100.0,
            step=0.5,
            value=10.0,
        )

        speed_label = field_label("Speed Scale")
        self._speed_spin = double_spinbox(
            decimals=2,
            min_value=0.01,
            max_value=1.0,
            step=0.05,
            value=0.3,
        )

        layout.addWidget(step_label, 0, 0)
        layout.addWidget(self._step_spin, 0, 1)
        layout.addWidget(speed_label, 1, 0)
        layout.addWidget(self._speed_spin, 1, 1)

        return group

    def _build_pose_group(self) -> QGroupBox:
        group = QGroupBox("Current End-Effector Pose")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(8)

        self._pose_labels = {}
        for i, axis in enumerate(["X", "Y", "Z", "RX", "RY", "RZ"]):
            label = field_label(f"{axis}:")
            value = QLabel("-")
            value.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            self._pose_labels[axis.lower()] = value

            row = i // 3
            col = (i % 3) * 2
            layout.addWidget(label, row, col)
            layout.addWidget(value, row, col + 1)

        return group

    def _build_jog_group(self) -> QGroupBox:
        group = QGroupBox("Cartesian Jog Controls")
        layout = QVBoxLayout(group)
        layout.setSpacing(12)

        trans_group = QGroupBox("Translation")
        trans_layout = QGridLayout(trans_group)
        trans_layout.setHorizontalSpacing(8)
        trans_layout.setVerticalSpacing(8)

        self._btn_x_minus = action_button("← -X", variant="primary")
        self._btn_x_plus = action_button("+X →", variant="primary")
        self._btn_y_minus = action_button("← -Y", variant="primary")
        self._btn_y_plus = action_button("+Y →", variant="primary")
        self._btn_z_minus = action_button("↓ -Z", variant="primary")
        self._btn_z_plus = action_button("+Z ↑", variant="primary")

        qt_connect(self._btn_x_minus.clicked, lambda: self._send_step("x", -1.0))
        qt_connect(self._btn_x_plus.clicked, lambda: self._send_step("x", +1.0))
        qt_connect(self._btn_y_minus.clicked, lambda: self._send_step("y", -1.0))
        qt_connect(self._btn_y_plus.clicked, lambda: self._send_step("y", +1.0))
        qt_connect(self._btn_z_minus.clicked, lambda: self._send_step("z", -1.0))
        qt_connect(self._btn_z_plus.clicked, lambda: self._send_step("z", +1.0))

        for btn in [
            self._btn_x_minus,
            self._btn_x_plus,
            self._btn_y_minus,
            self._btn_y_plus,
            self._btn_z_minus,
            self._btn_z_plus,
        ]:
            btn.setEnabled(False)

        trans_layout.addWidget(self._btn_x_minus, 0, 0)
        trans_layout.addWidget(self._btn_x_plus, 0, 1)
        trans_layout.addWidget(self._btn_y_minus, 1, 0)
        trans_layout.addWidget(self._btn_y_plus, 1, 1)
        trans_layout.addWidget(self._btn_z_minus, 2, 0)
        trans_layout.addWidget(self._btn_z_plus, 2, 1)

        layout.addWidget(trans_group)

        rot_group = QGroupBox("Rotation")
        rot_layout = QGridLayout(rot_group)
        rot_layout.setHorizontalSpacing(8)
        rot_layout.setVerticalSpacing(8)

        self._btn_rx_minus = action_button("← -RX", variant="primary")
        self._btn_rx_plus = action_button("+RX →", variant="primary")
        self._btn_ry_minus = action_button("← -RY", variant="primary")
        self._btn_ry_plus = action_button("+RY →", variant="primary")
        self._btn_rz_minus = action_button("← -RZ", variant="primary")
        self._btn_rz_plus = action_button("+RZ →", variant="primary")

        qt_connect(self._btn_rx_minus.clicked, lambda: self._send_step("rx", -1.0))
        qt_connect(self._btn_rx_plus.clicked, lambda: self._send_step("rx", +1.0))
        qt_connect(self._btn_ry_minus.clicked, lambda: self._send_step("ry", -1.0))
        qt_connect(self._btn_ry_plus.clicked, lambda: self._send_step("ry", +1.0))
        qt_connect(self._btn_rz_minus.clicked, lambda: self._send_step("rz", -1.0))
        qt_connect(self._btn_rz_plus.clicked, lambda: self._send_step("rz", +1.0))

        for btn in [
            self._btn_rx_minus,
            self._btn_rx_plus,
            self._btn_ry_minus,
            self._btn_ry_plus,
            self._btn_rz_minus,
            self._btn_rz_plus,
        ]:
            btn.setEnabled(False)

        rot_layout.addWidget(self._btn_rx_minus, 0, 0)
        rot_layout.addWidget(self._btn_rx_plus, 0, 1)
        rot_layout.addWidget(self._btn_ry_minus, 1, 0)
        rot_layout.addWidget(self._btn_ry_plus, 1, 1)
        rot_layout.addWidget(self._btn_rz_minus, 2, 0)
        rot_layout.addWidget(self._btn_rz_plus, 2, 1)

        layout.addWidget(rot_group)

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
        self._log_text.setMaximumHeight(180)
        set_role(self._log_text, "log")
        layout.addWidget(self._log_text)

        return group

    def _setup_status_timer(self) -> None:
        self._status_timer = QTimer()
        qt_connect(self._status_timer.timeout, self._update_status)
        self._status_timer.start(1000)

    def _setup_pose_timer(self) -> None:
        self._pose_timer = QTimer()
        qt_connect(self._pose_timer.timeout, self._update_pose)
        self._pose_timer.start(500)

    def _update_status(self) -> None:
        is_connected = bool(self._client and self._client.is_connected())
        if is_connected != self._connected:
            self._set_connection_ui(is_connected)
            if is_connected:
                self._log_message("Connection", "Connected to MoveIt")
            else:
                self._log_message("Connection", "Disconnected")

            self._update_pose()

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

            for btn in [
                self._btn_x_minus,
                self._btn_x_plus,
                self._btn_y_minus,
                self._btn_y_plus,
                self._btn_z_minus,
                self._btn_z_plus,
                self._btn_rx_minus,
                self._btn_rx_plus,
                self._btn_ry_minus,
                self._btn_ry_plus,
                self._btn_rz_minus,
                self._btn_rz_plus,
            ]:
                btn.setEnabled(True)
            return

        self._status_text.setText("Disconnected")
        set_status(self._status_indicator, "disconnected")
        set_status(self._status_text, "disconnected")

        self._connect_button.setText("Connect")
        set_variant(self._connect_button, "primary")

        for btn in [
            self._btn_x_minus,
            self._btn_x_plus,
            self._btn_y_minus,
            self._btn_y_plus,
            self._btn_z_minus,
            self._btn_z_plus,
            self._btn_rx_minus,
            self._btn_rx_plus,
            self._btn_ry_minus,
            self._btn_ry_plus,
            self._btn_rz_minus,
            self._btn_rz_plus,
        ]:
            btn.setEnabled(False)

    def _update_pose(self) -> None:
        if not self._connected:
            for label in self._pose_labels.values():
                if label.text() != "-":
                    label.setText("-")
            return

        try:
            pose = self._client.get_current_pose()
            self._pose_labels["x"].setText(f"{pose['x']*1000:.1f} mm")
            self._pose_labels["y"].setText(f"{pose['y']*1000:.1f} mm")
            self._pose_labels["z"].setText(f"{pose['z']*1000:.1f} mm")
            self._pose_labels["rx"].setText(f"{pose['rx']:.3f} rad")
            self._pose_labels["ry"].setText(f"{pose['ry']:.3f} rad")
            self._pose_labels["rz"].setText(f"{pose['rz']:.3f} rad")
        except Exception:  # noqa: BLE001
            for label in self._pose_labels.values():
                if label.text() != "-":
                    label.setText("-")

    def _send_step(self, axis: str, direction: float) -> None:
        if not self._connected:
            self._log_message("Error", "Not connected")
            return

        step_mm = float(self._step_spin.value())
        speed_scale = float(self._speed_spin.value())

        if axis in ("x", "y", "z"):
            step_m = (step_mm / 1000.0) * direction
            unit = "mm"
        else:
            step_m = (step_mm / 1000.0) * direction
            unit = "rad"

        self._log_message(
            f"Jog {axis.upper()}",
            f"step={step_mm * direction:.1f} {unit}, speed={speed_scale:.2f}",
        )

        try:
            success = self._client.send_cartesian_step(axis, step_m, speed_scale)
            if success:
                self._log_message(f"Jog {axis.upper()}", "Command executed")
            else:
                self._log_message("Error", f"Jog {axis.upper()} failed (planning/execution)")
        except Exception as exc:  # noqa: BLE001
            self._log_message("Error", f"Jog failed: {exc}")

        self._update_pose()

    def _log_message(self, source: str, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log_text.append(f"[{timestamp}] {source}: {message}")

        scrollbar = self._log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _clear_log(self) -> None:
        self._log_text.clear()
        self._log_message("System", "Log cleared")

    def _toggle_connection(self) -> None:
        if self._connected:
            self._log_message("Connection", "Disconnecting...")
            try:
                self._client.disconnect()
            finally:
                self._update_status()
            return

        self._log_message("Connection", "Connecting to MoveIt...")
        try:
            ok = self._client.connect()
        except Exception as exc:  # noqa: BLE001
            ok = False
            self._log_message("Error", f"Connect failed: {exc}")

        if not ok:
            self._log_message("Error", "Connect failed")

        self._update_status()
