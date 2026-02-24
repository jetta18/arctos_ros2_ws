"""Motor settings widget for configuring MKS Servo 42D/57D motors via CAN bus.

Each axis (X/Y/Z/A/B/C) gets its own sub-tab. Every parameter row has a
dedicated "Send" button that immediately transmits the single CAN command
and persists the value via SettingsManager.
"""

from __future__ import annotations

import logging
from typing import Any, Optional

from PyQt5.QtWidgets import (
    QCheckBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QScrollArea,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ...backend.settings_manager import SettingsManager
from ...ui.theme import (
    CARD_SPACING,
    OUTER_MARGIN,
    WIDGET_SPACING,
    set_role,
    set_status,
    set_variant,
)
from ...ui.widgets import (
    action_button,
    combo_box,
    connect as qt_connect,
    double_spinbox,
    field_label,
    int_spinbox,
    status_indicator_row,
)
from .motor_settings_client_protocol import MotorSettingsClient

logger = logging.getLogger(__name__)

# Axis definitions
AXIS_NAMES: tuple[str, ...] = ("X", "Y", "Z", "A", "B", "C")

# Driver type per axis
_DRIVER_57D_AXES: frozenset[str] = frozenset({"X", "Y"})
_DRIVER_42D_AXES: frozenset[str] = frozenset({"Z", "A", "B", "C"})

# Current limits (mA)
_MAX_CURRENT_57D = 5200
_MAX_CURRENT_42D = 3000
_CURRENT_STEP = 100

# Work mode options
_WORK_MODES = [
    "CR_OPEN",
    "CR_CLOSE",
    "CR_vFOC",
    "SR_OPEN",
    "SR_CLOSE",
    "SR_vFOC",
]
_WORK_MODE_CODES = {name: i for i, name in enumerate(_WORK_MODES)}

# Direction options
_DIRECTIONS = ["CW", "CCW"]
_DIR_CODES = {"CW": 0, "CCW": 1}

# Homing speed options (RPM)
_HOMING_SPEEDS = [30, 60, 90, 120, 150, 180, 300, 600, 1200, 3000]

# Holding current percentage options (code 0–8 → 10%–90%)
_HOLD_CURRENT_LABELS = [f"{(i + 1) * 10}%" for i in range(9)]

# Micro-step options
_MICROSTEP_OPTIONS = [1, 2, 4, 8, 16, 32, 64, 128, 256]

# CAN bitrate options
_CAN_BITRATES = ["125K", "250K", "500K", "1M"]
_CAN_BITRATE_CODES = {"125K": 0, "250K": 1, "500K": 2, "1M": 3}

# Default CAN interface settings
_DEFAULT_CHANNEL = "/dev/ttyACM0"
_DEFAULT_BITRATE = "500K"

# Bitrate label → bps
_BITRATE_BPS: dict[str, int] = {
    "125K": 125000,
    "250K": 250000,
    "500K": 500000,
    "1M": 1000000,
}


class MotorSettingsWidget(QWidget):
    """Motor settings tab: CAN connection + per-axis parameter forms.

    Uses a :class:`MotorSettingsClient` to stay independent from the CAN
    implementation. Settings are persisted via :class:`SettingsManager`.
    """

    def __init__(
        self,
        settings_manager: SettingsManager,
        client: Optional[MotorSettingsClient] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        """Initializes the widget.

        Args:
            settings_manager: Shared settings persistence backend.
            client: Motor settings CAN client. May be None initially.
            parent: Optional parent widget.
        """
        super().__init__(parent)
        self._settings = settings_manager
        self._client = client

        self._build_ui()
        self._load_all_settings()

    def set_client(self, client: MotorSettingsClient) -> None:
        """Replaces the active client (e.g. after CAN connection).

        Args:
            client: New motor settings client instance.
        """
        self._client = client

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        """Build the ui."""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN)
        main_layout.setSpacing(CARD_SPACING)

        main_layout.addWidget(self._build_can_connection_group())

        axis_tabs = QTabWidget()
        axis_tabs.setObjectName("subTabs")

        self._axis_forms: dict[str, _AxisForm] = {}
        for axis in AXIS_NAMES:
            is_57d = axis in _DRIVER_57D_AXES
            form = _AxisForm(axis, is_57d, self._settings, self._get_client)
            self._axis_forms[axis] = form
            driver_label = "57D" if is_57d else "42D"
            axis_tabs.addTab(form, f"{axis}  ({driver_label})")

        main_layout.addWidget(axis_tabs)

    def _build_can_connection_group(self) -> QGroupBox:
        """Build the can connection group."""
        group = QGroupBox("CAN Interface  (slcan)")
        layout = QVBoxLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        # Status row
        status_row = QHBoxLayout()
        status_row.setSpacing(WIDGET_SPACING)
        self._can_dot, self._can_text = status_indicator_row("Disconnected")
        status_row.addWidget(self._can_dot)
        status_row.addWidget(self._can_text)
        status_row.addStretch(1)
        layout.addLayout(status_row)

        # Connection parameters
        form = QFormLayout()
        form.setSpacing(WIDGET_SPACING)

        # Channel: combo populated by refresh, plus a small refresh button
        channel_row = QHBoxLayout()
        channel_row.setSpacing(WIDGET_SPACING)
        self._channel_combo = combo_box(items=[])
        self._channel_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._btn_refresh = action_button("↺", variant=None)
        self._btn_refresh.setFixedWidth(36)
        self._btn_refresh.setToolTip("Scan for available serial devices")
        qt_connect(self._btn_refresh.clicked, self._on_refresh_devices)
        channel_row.addWidget(self._channel_combo)
        channel_row.addWidget(self._btn_refresh)
        form.addRow(field_label("Device"), channel_row)

        self._bitrate_combo = combo_box(items=_CAN_BITRATES)
        saved_bitrate = self._settings.get("can.bitrate", _DEFAULT_BITRATE)
        bidx = self._bitrate_combo.findText(saved_bitrate)
        if bidx >= 0:
            self._bitrate_combo.setCurrentIndex(bidx)
        form.addRow(field_label("Bitrate"), self._bitrate_combo)

        layout.addLayout(form)

        # Connect / Disconnect buttons
        btn_row = QHBoxLayout()
        btn_row.setSpacing(WIDGET_SPACING)
        self._btn_connect = action_button("Connect", variant="primary")
        self._btn_disconnect = action_button("Disconnect", variant="danger")
        self._btn_disconnect.setEnabled(False)
        qt_connect(self._btn_connect.clicked, self._on_connect)
        qt_connect(self._btn_disconnect.clicked, self._on_disconnect)
        btn_row.addWidget(self._btn_connect)
        btn_row.addWidget(self._btn_disconnect)
        btn_row.addStretch(1)
        layout.addLayout(btn_row)

        # Populate device list immediately
        self._on_refresh_devices()

        return group

    # ------------------------------------------------------------------
    # Settings load
    # ------------------------------------------------------------------

    def _load_all_settings(self) -> None:
        """Load the all settings."""
        for form in self._axis_forms.values():
            form.load_settings()

    # ------------------------------------------------------------------
    # CAN connection
    # ------------------------------------------------------------------

    def _get_client(self) -> Optional[MotorSettingsClient]:
        """Return the client."""
        return self._client

    def _on_refresh_devices(self) -> None:
        """Scans /dev for ttyACM* and ttyUSB* devices and repopulates the combo."""
        import glob
        devices = sorted(
            glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        )
        saved = self._settings.get("can.channel", _DEFAULT_CHANNEL)
        self._channel_combo.clear()
        if devices:
            for dev in devices:
                self._channel_combo.addItem(dev)
            target = saved if saved in devices else devices[0]
            self._channel_combo.setCurrentText(target)
        else:
            self._channel_combo.addItem(saved)
            self._channel_combo.setCurrentText(saved)

    def _on_connect(self) -> None:
        """Handle the connect event."""
        if self._client is None:
            self._set_can_status(False, "No client available")
            return

        channel = self._channel_combo.currentText()
        bitrate_str = self._bitrate_combo.currentText()
        bitrate_bps = _BITRATE_BPS.get(bitrate_str, 500000)

        self._settings.set("can.channel", channel)
        self._settings.set("can.bitrate", bitrate_str)
        self._settings.save()

        ok = self._client.connect("slcan", channel, bitrate_bps)
        self._set_can_status(ok, "Connected" if ok else "Connection failed")

    def _on_disconnect(self) -> None:
        """Handle the disconnect event."""
        if self._client is not None:
            self._client.disconnect()
        self._set_can_status(False, "Disconnected")

    def _set_can_status(self, connected: bool, message: str) -> None:
        """Set the can status."""
        status = "connected" if connected else "disconnected"
        set_status(self._can_dot, status)
        set_status(self._can_text, status)
        self._can_text.setText(message)
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)


# ------------------------------------------------------------------
# Per-axis form widget
# ------------------------------------------------------------------

class _AxisForm(QWidget):
    """Form widget for a single axis motor configuration.

    Each parameter row contains a value control and a dedicated "Send" button.
    """

    def __init__(
        self,
        axis: str,
        is_57d: bool,
        settings: SettingsManager,
        get_client,  # callable returning Optional[MotorSettingsClient]
        parent: Optional[QWidget] = None,
    ) -> None:
        """Initialize the instance."""
        super().__init__(parent)
        self._axis = axis
        self._is_57d = is_57d
        self._settings = settings
        self._get_client = get_client
        self._max_current = _MAX_CURRENT_57D if is_57d else _MAX_CURRENT_42D

        self._build_ui()

    def _settings_key(self, param: str) -> str:
        """Perform settings key."""
        return f"motor.{self._axis}.{param}"

    def _build_ui(self) -> None:
        """Build the ui."""
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN)
        layout.setSpacing(CARD_SPACING)

        layout.addWidget(self._build_identity_group())
        layout.addWidget(self._build_current_group())
        layout.addWidget(self._build_motion_group())
        layout.addWidget(self._build_homing_group())
        layout.addWidget(self._build_protection_group())
        layout.addWidget(self._build_can_group())
        layout.addWidget(self._build_read_group())

        save_all_btn = action_button("Save All to Motor", variant="success")
        qt_connect(save_all_btn.clicked, self._on_save_all_to_motor)
        layout.addWidget(save_all_btn)

        layout.addStretch()

        scroll.setWidget(content)
        outer.addWidget(scroll)

    # ------------------------------------------------------------------
    # Parameter group builders
    # ------------------------------------------------------------------

    def _build_identity_group(self) -> QGroupBox:
        """Build the identity group."""
        group = QGroupBox("Identity")
        layout = QFormLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        driver_label = "MKS Servo 57D (NEMA 23)" if self._is_57d else "MKS Servo 42D (NEMA 17)"
        info = QLabel(driver_label)
        set_role(info, "muted")
        layout.addRow(field_label("Driver"), info)

        self._can_id_spin = int_spinbox(min_value=1, max_value=2047, value=1)
        layout.addRow(
            field_label("CAN Node ID"),
            self._param_row(self._can_id_spin, "can_id"),
        )
        return group

    def _build_current_group(self) -> QGroupBox:
        """Build the current group."""
        group = QGroupBox("Current")
        layout = QFormLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        self._working_current_spin = int_spinbox(
            min_value=0,
            max_value=self._max_current,
            step=_CURRENT_STEP,
            value=1600 if not self._is_57d else 3200,
            suffix=" mA",
        )
        layout.addRow(
            field_label(f"Working current (0–{self._max_current} mA)"),
            self._param_row(self._working_current_spin, "working_current"),
        )

        self._hold_current_combo = combo_box(items=_HOLD_CURRENT_LABELS)
        layout.addRow(
            field_label("Holding current %"),
            self._param_row(self._hold_current_combo, "hold_current_pct"),
        )
        return group

    def _build_motion_group(self) -> QGroupBox:
        """Build the motion group."""
        group = QGroupBox("Motion")
        layout = QFormLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        self._work_mode_combo = combo_box(items=_WORK_MODES)
        layout.addRow(
            field_label("Work mode"),
            self._param_row(self._work_mode_combo, "work_mode"),
        )

        self._run_dir_combo = combo_box(items=_DIRECTIONS)
        layout.addRow(
            field_label("Run direction"),
            self._param_row(self._run_dir_combo, "run_direction"),
        )

        self._microstep_combo = combo_box(items=[str(v) for v in _MICROSTEP_OPTIONS])
        layout.addRow(
            field_label("Micro-steps"),
            self._param_row(self._microstep_combo, "microsteps"),
        )

        self._subdiv_interp_check = QCheckBox("Enable")
        layout.addRow(
            field_label("Subdivision interpolation"),
            self._param_row(self._subdiv_interp_check, "subdiv_interpolation"),
        )
        return group

    def _build_homing_group(self) -> QGroupBox:
        """Build the homing group."""
        group = QGroupBox("Homing")
        layout = QFormLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        self._home_dir_combo = combo_box(items=_DIRECTIONS)
        self._connect_autosave(self._home_dir_combo, "homing_direction")
        layout.addRow(field_label("Homing direction"), self._home_dir_combo)

        self._home_speed_combo = combo_box(items=[str(v) for v in _HOMING_SPEEDS])
        self._connect_autosave(self._home_speed_combo, "homing_speed")
        layout.addRow(field_label("Homing speed (RPM)"), self._home_speed_combo)

        self._endstop_level_combo = combo_box(items=["Low", "High"])
        self._connect_autosave(self._endstop_level_combo, "endstop_trigger_level")
        layout.addRow(field_label("Endstop trigger level"), self._endstop_level_combo)

        self._endstop_limit_check = QCheckBox("Enable")
        self._connect_autosave(self._endstop_limit_check, "endstop_limit_enable")
        layout.addRow(field_label("Endstop limit"), self._endstop_limit_check)

        send_home_btn = action_button("Send Home Settings", variant="primary")
        qt_connect(send_home_btn.clicked, self._on_send_home)
        layout.addRow("", send_home_btn)

        return group

    def _build_protection_group(self) -> QGroupBox:
        """Build the protection group."""
        group = QGroupBox("Protection")
        layout = QFormLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        self._stall_protection_check = QCheckBox("Enable")
        layout.addRow(
            field_label("Stall protection"),
            self._param_row(self._stall_protection_check, "stall_protection"),
        )
        return group

    def _build_can_group(self) -> QGroupBox:
        """Build the can group."""
        group = QGroupBox("CAN Settings")
        layout = QFormLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        self._can_bitrate_combo = combo_box(items=_CAN_BITRATES)
        layout.addRow(
            field_label("CAN bitrate"),
            self._param_row(self._can_bitrate_combo, "can_bitrate"),
        )
        return group

    def _build_read_group(self) -> QGroupBox:
        """Build the read group."""
        group = QGroupBox("Read from Motor")
        layout = QVBoxLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        btn_read = action_button("Read Motor Status", variant=None)
        qt_connect(btn_read.clicked, self._on_read_motor)
        layout.addWidget(btn_read)

        self._read_result_label = QLabel("—")
        set_role(self._read_result_label, "muted")
        self._read_result_label.setWordWrap(True)
        layout.addWidget(self._read_result_label)

        return group

    # ------------------------------------------------------------------
    # Helper: parameter row with Send button
    # ------------------------------------------------------------------

    def _param_row(self, control: QWidget, command: str) -> QWidget:
        """Wraps *control* in a row widget with a dedicated Send button.

        The control's change signal is connected to auto-save (persist to YAML
        immediately on every change). The Send button additionally transmits the
        value to the motor via CAN.

        Args:
            control: The input widget (spinbox, combo, checkbox).
            command: Command identifier string passed to send_command().

        Returns:
            A QWidget containing the control and Send button side by side.
        """
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(WIDGET_SPACING)

        control.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        row_layout.addWidget(control)

        self._connect_autosave(control, command)

        send_btn = action_button("Send", variant="primary")
        send_btn.setFixedWidth(70)
        qt_connect(
            send_btn.clicked,
            lambda checked=False, c=command, w=control: self._on_send(c, w),
        )
        row_layout.addWidget(send_btn)

        return row

    def _connect_autosave(self, control: QWidget, command: str) -> None:
        """Connects the appropriate change signal of *control* to auto-save.

        Args:
            control: The input widget.
            command: Settings key suffix to persist under.
        """
        from PyQt5.QtWidgets import QAbstractSpinBox, QComboBox, QCheckBox

        if isinstance(control, QCheckBox):
            qt_connect(
                control.stateChanged,
                lambda _state, c=command, w=control: self._autosave(c, w),
            )
        elif isinstance(control, QComboBox):
            qt_connect(
                control.currentTextChanged,
                lambda _text, c=command, w=control: self._autosave(c, w),
            )
        elif isinstance(control, QAbstractSpinBox):
            qt_connect(
                control.valueChanged,
                lambda _val, c=command, w=control: self._autosave(c, w),
            )

    def _autosave(self, command: str, control: QWidget) -> None:
        """Persists the current control value to SettingsManager without sending CAN.

        Args:
            command: Settings key suffix.
            control: The input widget to read from.
        """
        value = self._read_control_value(control)
        self._settings.set(self._settings_key(command), value)
        self._settings.save()

    # ------------------------------------------------------------------
    # Settings persistence
    # ------------------------------------------------------------------

    def load_settings(self) -> None:
        """Loads persisted values into all controls for this axis."""
        self._can_id_spin.setValue(
            int(self._settings.get(self._settings_key("can_id"), 1))
        )
        self._working_current_spin.setValue(
            int(self._settings.get(
                self._settings_key("working_current"),
                1600 if not self._is_57d else 3200,
            ))
        )
        hold_pct = self._settings.get(self._settings_key("hold_current_pct"), "50%")
        hold_idx = self._hold_current_combo.findText(str(hold_pct))
        if hold_idx >= 0:
            self._hold_current_combo.setCurrentIndex(hold_idx)

        work_mode = self._settings.get(self._settings_key("work_mode"), "SR_vFOC")
        wm_idx = self._work_mode_combo.findText(work_mode)
        if wm_idx >= 0:
            self._work_mode_combo.setCurrentIndex(wm_idx)

        run_dir = self._settings.get(self._settings_key("run_direction"), "CW")
        rd_idx = self._run_dir_combo.findText(run_dir)
        if rd_idx >= 0:
            self._run_dir_combo.setCurrentIndex(rd_idx)

        microstep = self._settings.get(self._settings_key("microsteps"), "16")
        ms_idx = self._microstep_combo.findText(str(microstep))
        if ms_idx >= 0:
            self._microstep_combo.setCurrentIndex(ms_idx)

        self._subdiv_interp_check.setChecked(
            bool(self._settings.get(self._settings_key("subdiv_interpolation"), True))
        )

        home_dir = self._settings.get(self._settings_key("homing_direction"), "CW")
        hd_idx = self._home_dir_combo.findText(home_dir)
        if hd_idx >= 0:
            self._home_dir_combo.setCurrentIndex(hd_idx)

        home_speed = self._settings.get(self._settings_key("homing_speed"), "60")
        hs_idx = self._home_speed_combo.findText(str(home_speed))
        if hs_idx >= 0:
            self._home_speed_combo.setCurrentIndex(hs_idx)

        endstop_level = self._settings.get(self._settings_key("endstop_trigger_level"), "Low")
        el_idx = self._endstop_level_combo.findText(endstop_level)
        if el_idx >= 0:
            self._endstop_level_combo.setCurrentIndex(el_idx)

        self._endstop_limit_check.setChecked(
            bool(self._settings.get(self._settings_key("endstop_limit_enable"), False))
        )
        self._stall_protection_check.setChecked(
            bool(self._settings.get(self._settings_key("stall_protection"), False))
        )

        can_bitrate = self._settings.get(self._settings_key("can_bitrate"), "500K")
        cb_idx = self._can_bitrate_combo.findText(can_bitrate)
        if cb_idx >= 0:
            self._can_bitrate_combo.setCurrentIndex(cb_idx)

    # ------------------------------------------------------------------
    # Send / Read handlers
    # ------------------------------------------------------------------

    def _on_send(self, command: str, control: QWidget) -> None:
        """Handle the send event."""
        value = self._read_control_value(control)
        self._settings.set(self._settings_key(command), value)
        self._settings.save()

        client = self._get_client()
        if client is None or not client.is_connected():
            logger.info(
                "Axis %s: saved %s=%r (CAN not connected, not sent)", self._axis, command, value
            )
            return

        ok = client.send_command(self._axis, command, value)
        if ok:
            logger.info("Axis %s: sent %s=%r — OK", self._axis, command, value)
        else:
            logger.warning("Axis %s: sent %s=%r — FAILED", self._axis, command, value)

    def _on_send_home(self) -> None:
        """Sends all homing parameters as a single combined set_home command."""
        home_dict = {
            "home_trig": self._endstop_level_combo.currentText(),
            "home_dir": self._home_dir_combo.currentText(),
            "home_speed": int(self._home_speed_combo.currentText()),
            "end_limit": self._endstop_limit_check.isChecked(),
        }
        self._settings.set(self._settings_key("homing_direction"), home_dict["home_dir"])
        self._settings.set(self._settings_key("homing_speed"), home_dict["home_speed"])
        self._settings.set(self._settings_key("endstop_trigger_level"), home_dict["home_trig"])
        self._settings.set(self._settings_key("endstop_limit_enable"), home_dict["end_limit"])
        self._settings.save()

        client = self._get_client()
        if client is None or not client.is_connected():
            logger.info("Axis %s: saved home settings (CAN not connected)", self._axis)
            return

        ok = client.send_command(self._axis, "set_home", home_dict)
        if ok:
            logger.info("Axis %s: set_home → OK", self._axis)
        else:
            logger.warning("Axis %s: set_home → FAILED", self._axis)

    def _on_save_all_to_motor(self) -> None:
        """Sends all current parameter values to the motor sequentially."""
        commands = [
            ("can_id", self._can_id_spin),
            ("working_current", self._working_current_spin),
            ("hold_current_pct", self._hold_current_combo),
            ("work_mode", self._work_mode_combo),
            ("run_direction", self._run_dir_combo),
            ("microsteps", self._microstep_combo),
            ("subdiv_interpolation", self._subdiv_interp_check),
            ("stall_protection", self._stall_protection_check),
            ("can_bitrate", self._can_bitrate_combo),
        ]
        for command, control in commands:
            self._on_send(command, control)

        self._on_send_home()
        logger.info("Axis %s: Save All complete", self._axis)

    def _on_read_motor(self) -> None:
        """Handle the read motor event."""
        client = self._get_client()
        if client is None or not client.is_connected():
            self._read_result_label.setText("Not connected")
            return

        try:
            status = client.read_motor_status(self._axis)
            lines = [f"{k}: {v}" for k, v in status.items()]
            self._read_result_label.setText("\n".join(lines) if lines else "No data")
        except Exception as exc:
            self._read_result_label.setText(f"Error: {exc}")
            logger.exception("Axis %s: read_motor_status failed", self._axis)

    @staticmethod
    def _read_control_value(control: QWidget) -> Any:
        """Extracts the current value from any supported control widget."""
        from PyQt5.QtWidgets import QAbstractSpinBox, QComboBox, QCheckBox

        if isinstance(control, QCheckBox):
            return control.isChecked()
        if isinstance(control, QComboBox):
            return control.currentText()
        if isinstance(control, QAbstractSpinBox):
            return control.value()
        return None
