"""Homing widget for the Arctos GUI.

Provides:
  - Single axis homing (to endstop or to home position)
  - Full robot homing with configurable axis sequence
  - Offset tuning wizard (jog to position, capture offset)
  - Per-axis homing progress visualization
  - Emergency stop
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListView,
    QScrollArea,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ...backend.app_state import AXIS_NAMES
from ...backend.settings_manager import SettingsManager
from ...ui.theme import (
    CARD_SPACING,
    OUTER_MARGIN,
    WIDGET_SPACING,
    set_role,
    set_status,
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
from .homing_client_protocol import HomingClient, HomingStatus

_POLL_INTERVAL_MS = 200
_NUM_AXES = len(AXIS_NAMES)
_DIRECTION_LABELS = ["MIN", "MAX"]
_DEFAULT_VELOCITY_RAD_S = 0.5
_DEFAULT_HOMING_SEQUENCE = list(range(_NUM_AXES))

_SETTINGS_PREFIX = "homing"


class HomingWidget(QWidget):
    """Homing control widget with progress visualization and offset tuning.

    Uses a `HomingClient` to keep the Qt layer independent from ROS specifics.
    """

    def __init__(
        self,
        client: HomingClient,
        settings_manager: Optional[SettingsManager] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        """Initializes the homing widget.

        Args:
            client: Homing client backend.
            settings_manager: Shared settings persistence backend.
            parent: Optional parent widget.
        """
        super().__init__(parent)
        self._client = client
        self._settings = settings_manager or SettingsManager()
        self._connected = False
        self._homing_sequence = list(_DEFAULT_HOMING_SEQUENCE)
        self._home_all_running = False
        self._home_all_index = 0
        self._loading_settings = False

        self._build_ui()
        self._load_settings()
        self._connect_auto_save()
        self._setup_poll_timer()
        self._log_message("System", "Homing widget initialized")

    def _build_ui(self) -> None:
        """Build the ui."""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(
            OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN,
        )
        main_layout.setSpacing(CARD_SPACING)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QScrollArea.NoFrame)

        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(CARD_SPACING)

        scroll_layout.addWidget(self._build_status_group())
        scroll_layout.addWidget(self._build_progress_group())
        scroll_layout.addWidget(self._build_single_axis_group())
        scroll_layout.addWidget(self._build_offset_tuning_group())
        scroll_layout.addWidget(self._build_full_homing_group())
        scroll_layout.addWidget(self._build_emergency_group())
        scroll_layout.addWidget(self._build_log_group())
        scroll_layout.addStretch()

        scroll_area.setWidget(scroll_widget)
        main_layout.addWidget(scroll_area)

    # ------------------------------------------------------------------ #
    # Settings persistence                                                 #
    # ------------------------------------------------------------------ #

    def _settings_key(self, key: str) -> str:
        """Returns a dot-separated settings key under the homing prefix."""
        return f"{_SETTINGS_PREFIX}.{key}"

    def _axis_settings_key(self, axis_name: str, key: str) -> str:
        """Returns a dot-separated per-axis settings key."""
        return f"{_SETTINGS_PREFIX}.{axis_name}.{key}"

    def _load_settings(self) -> None:
        """Loads persisted homing settings into all controls."""
        self._loading_settings = True
        try:
            axis_idx = self._axis_combo.currentIndex()
            axis_name = AXIS_NAMES[axis_idx]

            direction = int(self._settings.get(
                self._axis_settings_key(axis_name, "direction"), 0,
            ))
            if 0 <= direction < len(_DIRECTION_LABELS):
                self._direction_combo.setCurrentIndex(direction)

            velocity = float(self._settings.get(
                self._axis_settings_key(axis_name, "velocity_rad_s"),
                _DEFAULT_VELOCITY_RAD_S,
            ))
            self._velocity_spin.setValue(velocity)

            full_vel = float(self._settings.get(
                self._settings_key("full_velocity_rad_s"),
                _DEFAULT_VELOCITY_RAD_S,
            ))
            self._full_velocity_spin.setValue(full_vel)

            seq_idx = int(self._settings.get(
                self._settings_key("sequence_index"), 0,
            ))
            if 0 <= seq_idx < self._sequence_combo.count():
                self._sequence_combo.setCurrentIndex(seq_idx)
        finally:
            self._loading_settings = False

    def _connect_auto_save(self) -> None:
        """Connects control signals to auto-save."""
        qt_connect(
            self._axis_combo.currentIndexChanged,
            self._on_single_axis_changed,
        )
        qt_connect(
            self._direction_combo.currentIndexChanged,
            self._on_setting_changed,
        )
        qt_connect(
            self._velocity_spin.valueChanged,
            self._on_setting_changed,
        )
        qt_connect(
            self._full_velocity_spin.valueChanged,
            self._on_full_setting_changed,
        )
        qt_connect(
            self._sequence_combo.currentIndexChanged,
            self._on_full_setting_changed,
        )

    def _on_single_axis_changed(self) -> None:
        """Reload per-axis settings when axis selection changes."""
        if self._loading_settings:
            return
        self._loading_settings = True
        try:
            axis_name = AXIS_NAMES[self._axis_combo.currentIndex()]
            direction = int(self._settings.get(
                self._axis_settings_key(axis_name, "direction"), 0,
            ))
            if 0 <= direction < len(_DIRECTION_LABELS):
                self._direction_combo.setCurrentIndex(direction)
            velocity = float(self._settings.get(
                self._axis_settings_key(axis_name, "velocity_rad_s"),
                _DEFAULT_VELOCITY_RAD_S,
            ))
            self._velocity_spin.setValue(velocity)
        finally:
            self._loading_settings = False

    def _on_setting_changed(self) -> None:
        """Auto-save per-axis homing settings on any control change."""
        if self._loading_settings:
            return
        axis_name = AXIS_NAMES[self._axis_combo.currentIndex()]
        self._settings.set(
            self._axis_settings_key(axis_name, "direction"),
            self._direction_combo.currentIndex(),
        )
        self._settings.set(
            self._axis_settings_key(axis_name, "velocity_rad_s"),
            self._velocity_spin.value(),
        )
        self._settings.save()

    def _on_full_setting_changed(self) -> None:
        """Auto-save full-homing settings on any control change."""
        if self._loading_settings:
            return
        self._settings.set(
            self._settings_key("full_velocity_rad_s"),
            self._full_velocity_spin.value(),
        )
        self._settings.set(
            self._settings_key("sequence_index"),
            self._sequence_combo.currentIndex(),
        )
        self._settings.save()

    # ------------------------------------------------------------------ #
    # Connection status                                                    #
    # ------------------------------------------------------------------ #

    def _build_status_group(self) -> QGroupBox:
        """Build the status group."""
        group = QGroupBox("Connection")
        layout = QHBoxLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        self._conn_dot, self._conn_text = status_indicator_row("Disconnected")
        layout.addWidget(self._conn_dot)
        layout.addWidget(self._conn_text)
        layout.addStretch(1)

        self._last_update_label = QLabel("Last update: —")
        set_role(self._last_update_label, "muted")
        layout.addWidget(self._last_update_label)

        return group

    # ------------------------------------------------------------------ #
    # Progress visualization                                               #
    # ------------------------------------------------------------------ #

    def _build_progress_group(self) -> QGroupBox:
        """Build the progress group."""
        group = QGroupBox("Homing Progress")
        layout = QGridLayout(group)
        layout.setSpacing(WIDGET_SPACING)
        layout.setContentsMargins(
            OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN,
        )

        layout.addWidget(field_label("Axis"), 0, 0)
        layout.addWidget(field_label("State"), 0, 1)
        layout.addWidget(field_label("Homed"), 0, 2)

        self._progress_dots: list[QLabel] = []
        self._progress_texts: list[QLabel] = []
        self._homed_labels: list[QLabel] = []

        for i, axis_name in enumerate(AXIS_NAMES):
            row = i + 1

            axis_label = QLabel(axis_name)
            set_role(axis_label, "fieldLabel")
            layout.addWidget(axis_label, row, 0)

            dot, text = status_indicator_row("IDLE", initial_status="connected")
            state_widget = QWidget()
            state_layout = QHBoxLayout(state_widget)
            state_layout.setContentsMargins(0, 0, 0, 0)
            state_layout.setSpacing(WIDGET_SPACING)
            state_layout.addWidget(dot)
            state_layout.addWidget(text)
            state_layout.addStretch()
            layout.addWidget(state_widget, row, 1)

            homed_label = QLabel("--")
            set_role(homed_label, "muted")
            layout.addWidget(homed_label, row, 2)

            self._progress_dots.append(dot)
            self._progress_texts.append(text)
            self._homed_labels.append(homed_label)

        return group

    # ------------------------------------------------------------------ #
    # Single axis homing                                                   #
    # ------------------------------------------------------------------ #

    def _build_single_axis_group(self) -> QGroupBox:
        """Build the single axis group."""
        group = QGroupBox("Single Axis Homing")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(CARD_SPACING)
        layout.setVerticalSpacing(WIDGET_SPACING)

        layout.addWidget(field_label("Axis"), 0, 0)
        self._axis_combo = combo_box(items=list(AXIS_NAMES))
        self._axis_combo.setView(QListView())
        layout.addWidget(self._axis_combo, 0, 1)

        layout.addWidget(field_label("Direction"), 1, 0)
        self._direction_combo = combo_box(items=_DIRECTION_LABELS)
        self._direction_combo.setView(QListView())
        layout.addWidget(self._direction_combo, 1, 1)

        layout.addWidget(field_label("Velocity [rad/s]"), 2, 0)
        self._velocity_spin = double_spinbox(
            decimals=3, min_value=0.01, max_value=10.0,
            step=0.1, value=_DEFAULT_VELOCITY_RAD_S,
        )
        layout.addWidget(self._velocity_spin, 2, 1)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(CARD_SPACING)

        self._btn_home_endstop = action_button(
            "Home to Endstop", variant="primary",
        )
        qt_connect(self._btn_home_endstop.clicked, self._on_home_to_endstop)
        self._btn_home_endstop.setEnabled(False)

        self._btn_home_position = action_button(
            "Home to Position", variant="primary",
        )
        qt_connect(self._btn_home_position.clicked, self._on_home_to_position)
        self._btn_home_position.setEnabled(False)

        btn_row.addWidget(self._btn_home_endstop)
        btn_row.addWidget(self._btn_home_position)
        layout.addLayout(btn_row, 3, 0, 1, 2)

        return group

    # ------------------------------------------------------------------ #
    # Offset tuning wizard                                                 #
    # ------------------------------------------------------------------ #

    def _build_offset_tuning_group(self) -> QGroupBox:
        """Build the offset tuning group."""
        group = QGroupBox("Offset Tuning Wizard")
        outer = QVBoxLayout(group)
        outer.setSpacing(CARD_SPACING)

        top = QGridLayout()
        top.setHorizontalSpacing(CARD_SPACING)
        top.setVerticalSpacing(WIDGET_SPACING)

        top.addWidget(field_label("Axis"), 0, 0)
        self._wiz_axis_combo = combo_box(items=list(AXIS_NAMES))
        self._wiz_axis_combo.setView(QListView())
        qt_connect(
            self._wiz_axis_combo.currentIndexChanged,
            self._on_wiz_axis_changed,
        )
        top.addWidget(self._wiz_axis_combo, 0, 1)

        top.addWidget(field_label("Stored Offset"), 0, 2)
        self._wiz_stored_offset = QLabel("--")
        self._wiz_stored_offset.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Preferred,
        )
        top.addWidget(self._wiz_stored_offset, 0, 3)

        outer.addLayout(top)

        step1 = QGroupBox("Step 1: Home to Endstop")
        s1_layout = QHBoxLayout(step1)
        s1_layout.setSpacing(CARD_SPACING)
        self._wiz_direction_combo = combo_box(items=_DIRECTION_LABELS)
        self._wiz_direction_combo.setView(QListView())
        s1_layout.addWidget(field_label("Direction"))
        s1_layout.addWidget(self._wiz_direction_combo)

        self._wiz_velocity_spin = double_spinbox(
            decimals=3, min_value=0.01, max_value=10.0,
            step=0.1, value=_DEFAULT_VELOCITY_RAD_S,
        )
        s1_layout.addWidget(field_label("Velocity [rad/s]"))
        s1_layout.addWidget(self._wiz_velocity_spin)

        self._btn_wiz_home_endstop = action_button(
            "Home to Endstop", variant="primary",
        )
        qt_connect(self._btn_wiz_home_endstop.clicked, self._on_wiz_home_endstop)
        self._btn_wiz_home_endstop.setEnabled(False)
        s1_layout.addWidget(self._btn_wiz_home_endstop)

        self._btn_wiz_reset_position = action_button(
            "Reset Position to Zero", variant="warning",
        )
        qt_connect(self._btn_wiz_reset_position.clicked, self._on_wiz_reset_position)
        self._btn_wiz_reset_position.setEnabled(False)
        s1_layout.addWidget(self._btn_wiz_reset_position)
        outer.addWidget(step1)

        step2 = QGroupBox("Step 2: Jog to Home Position")
        s2_layout = QGridLayout(step2)
        s2_layout.setHorizontalSpacing(CARD_SPACING)
        s2_layout.setVerticalSpacing(WIDGET_SPACING)

        s2_layout.addWidget(field_label("Current Position [steps]"), 0, 0)
        self._wiz_current_steps = QLabel("--")
        self._wiz_current_steps.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Preferred,
        )
        s2_layout.addWidget(self._wiz_current_steps, 0, 1, 1, 3)

        s2_layout.addWidget(field_label("Jog Distance [steps]"), 1, 0)
        self._wiz_jog_distance = int_spinbox(
            min_value=1, max_value=100000, step=100, value=500,
        )
        s2_layout.addWidget(self._wiz_jog_distance, 1, 1)

        self._btn_jog_minus = action_button("Jog -")
        qt_connect(self._btn_jog_minus.clicked, self._on_jog_minus)
        self._btn_jog_minus.setEnabled(False)
        s2_layout.addWidget(self._btn_jog_minus, 1, 2)

        self._btn_jog_plus = action_button("Jog +")
        qt_connect(self._btn_jog_plus.clicked, self._on_jog_plus)
        self._btn_jog_plus.setEnabled(False)
        s2_layout.addWidget(self._btn_jog_plus, 1, 3)

        s2_layout.addWidget(field_label("Jog Velocity [rad/s]"), 2, 0)
        self._wiz_jog_velocity = double_spinbox(
            decimals=3, min_value=0.01, max_value=5.0,
            step=0.05, value=0.2,
        )
        s2_layout.addWidget(self._wiz_jog_velocity, 2, 1)
        outer.addWidget(step2)

        step3 = QGroupBox("Step 3: Capture Offset")
        s3_layout = QGridLayout(step3)
        s3_layout.setHorizontalSpacing(CARD_SPACING)
        s3_layout.setVerticalSpacing(WIDGET_SPACING)

        s3_layout.addWidget(field_label("Calculated Offset [steps]"), 0, 0)
        self._wiz_calc_offset = QLabel("--")
        self._wiz_calc_offset.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Preferred,
        )
        s3_layout.addWidget(self._wiz_calc_offset, 0, 1)

        s3_layout.addWidget(field_label("Manual Offset [steps]"), 1, 0)
        self._wiz_manual_offset = int_spinbox(
            min_value=-1000000, max_value=1000000, step=100, value=0,
        )
        s3_layout.addWidget(self._wiz_manual_offset, 1, 1)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(CARD_SPACING)

        self._btn_capture_offset = action_button(
            "Capture Current Position", variant="primary",
        )
        qt_connect(self._btn_capture_offset.clicked, self._on_capture_offset)
        self._btn_capture_offset.setEnabled(False)

        self._btn_save_offset = action_button(
            "Save to Config", variant="success",
        )
        qt_connect(self._btn_save_offset.clicked, self._on_save_offset)
        self._btn_save_offset.setEnabled(False)

        btn_row.addWidget(self._btn_capture_offset)
        btn_row.addWidget(self._btn_save_offset)
        s3_layout.addLayout(btn_row, 2, 0, 1, 2)
        outer.addWidget(step3)

        return group

    # ------------------------------------------------------------------ #
    # Full robot homing                                                    #
    # ------------------------------------------------------------------ #

    def _build_full_homing_group(self) -> QGroupBox:
        """Build the full homing group."""
        group = QGroupBox("Full Robot Homing")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(CARD_SPACING)
        layout.setVerticalSpacing(WIDGET_SPACING)

        layout.addWidget(field_label("Velocity [rad/s]"), 0, 0)
        self._full_velocity_spin = double_spinbox(
            decimals=3, min_value=0.01, max_value=10.0,
            step=0.1, value=_DEFAULT_VELOCITY_RAD_S,
        )
        layout.addWidget(self._full_velocity_spin, 0, 1)

        layout.addWidget(field_label("Axis Sequence"), 1, 0)
        self._sequence_combo = combo_box(items=self._format_sequence_options())
        self._sequence_combo.setView(QListView())
        layout.addWidget(self._sequence_combo, 1, 1)

        self._full_progress_label = QLabel("Ready")
        set_role(self._full_progress_label, "muted")
        layout.addWidget(self._full_progress_label, 2, 0, 1, 2)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(CARD_SPACING)

        self._btn_home_all = action_button("Home All Axes", variant="primary")
        qt_connect(self._btn_home_all.clicked, self._on_home_all)
        self._btn_home_all.setEnabled(False)

        btn_row.addStretch()
        btn_row.addWidget(self._btn_home_all)
        btn_row.addStretch()
        layout.addLayout(btn_row, 3, 0, 1, 2)

        return group

    # ------------------------------------------------------------------ #
    # Emergency stop                                                       #
    # ------------------------------------------------------------------ #

    def _build_emergency_group(self) -> QGroupBox:
        """Build the emergency group."""
        group = QGroupBox("Emergency")
        layout = QHBoxLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        self._btn_stop = action_button("STOP ALL HOMING", variant="danger")
        qt_connect(self._btn_stop.clicked, self._on_stop_all)
        self._btn_stop.setEnabled(False)

        layout.addStretch()
        layout.addWidget(self._btn_stop)
        layout.addStretch()

        return group

    # ------------------------------------------------------------------ #
    # System log                                                           #
    # ------------------------------------------------------------------ #

    def _build_log_group(self) -> QGroupBox:
        """Build the log group."""
        group = QGroupBox("System Log")
        layout = QVBoxLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        controls = QHBoxLayout()
        controls.addStretch(1)
        self._log_clear = action_button("Clear")
        qt_connect(self._log_clear.clicked, self._clear_log)
        controls.addWidget(self._log_clear)
        layout.addLayout(controls)

        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumHeight(200)
        set_role(self._log_text, "log")
        layout.addWidget(self._log_text)

        return group

    # ------------------------------------------------------------------ #
    # Polling and status updates                                           #
    # ------------------------------------------------------------------ #

    def _setup_poll_timer(self) -> None:
        """Set up the poll timer."""
        self._poll_timer = QTimer()
        qt_connect(self._poll_timer.timeout, self._poll)
        self._poll_timer.start(_POLL_INTERVAL_MS)

    def _poll(self) -> None:
        """Poll the state."""
        if self._client is None:
            return

        connected = self._client.is_connected()
        self._update_connection_ui(connected)

        if not connected:
            return

        status = self._client.get_homing_status()
        self._update_progress_ui(status)
        self._update_wizard_position()
        self._check_home_all_progress(status)

        self._last_update_label.setText(
            f"Last update: {datetime.now().strftime('%H:%M:%S')}",
        )

    def _update_connection_ui(self, connected: bool) -> None:
        """Update the connection ui."""
        if connected == self._connected:
            return

        self._connected = connected
        enabled = connected

        if connected:
            set_status(self._conn_dot, "connected")
            set_status(self._conn_text, "connected")
            self._conn_text.setText("Connected")
        else:
            set_status(self._conn_dot, "disconnected")
            set_status(self._conn_text, "disconnected")
            self._conn_text.setText("Disconnected")

        self._btn_home_endstop.setEnabled(enabled)
        self._btn_home_position.setEnabled(enabled)
        self._btn_wiz_home_endstop.setEnabled(enabled)
        self._btn_wiz_reset_position.setEnabled(enabled)
        self._btn_jog_minus.setEnabled(enabled)
        self._btn_jog_plus.setEnabled(enabled)
        self._btn_capture_offset.setEnabled(enabled)
        self._btn_save_offset.setEnabled(enabled)
        self._btn_home_all.setEnabled(enabled)
        self._btn_stop.setEnabled(enabled)

    def _update_progress_ui(self, status: HomingStatus) -> None:
        """Update the progress ui."""
        for i in range(_NUM_AXES):
            axis_status = status.axes[i]
            state_name = axis_status.state

            if axis_status.active:
                dot_status = "disconnected"
            elif state_name == "COMPLETE":
                dot_status = "connected"
            else:
                dot_status = "connected"

            set_status(self._progress_dots[i], dot_status)
            set_status(self._progress_texts[i], dot_status)
            self._progress_texts[i].setText(state_name)

            if axis_status.is_homed:
                self._homed_labels[i].setText("YES")
                set_role(self._homed_labels[i], "accent")
            else:
                self._homed_labels[i].setText("NO")
                set_role(self._homed_labels[i], "muted")

    def _update_wizard_position(self) -> None:
        """Update the wizard position."""
        axis = self._wiz_axis_combo.currentIndex()
        try:
            steps = self._client.get_current_position_steps(axis)
            self._wiz_current_steps.setText(f"{steps:.0f}")
            self._wiz_calc_offset.setText(f"{steps:.0f}")
        except Exception:
            self._wiz_current_steps.setText("--")
            self._wiz_calc_offset.setText("--")

    # ------------------------------------------------------------------ #
    # Single axis handlers                                                 #
    # ------------------------------------------------------------------ #

    def _on_home_to_endstop(self) -> None:
        """Handle the home to endstop event."""
        axis = self._axis_combo.currentIndex()
        direction = self._direction_combo.currentIndex()
        velocity = self._velocity_spin.value()
        axis_name = AXIS_NAMES[axis]

        self._log_message(
            f"Home {axis_name}",
            f"To endstop {_DIRECTION_LABELS[direction]}, vel={velocity:.3f} rad/s",
        )

        try:
            ok, msg = self._client.home_axis_to_endstop(axis, direction, velocity)
            if ok:
                self._log_message(f"Home {axis_name}", "Started")
            else:
                self._log_message(f"Home {axis_name}", f"Failed: {msg}")
        except Exception as exc:
            self._log_message("Error", f"Home to endstop failed: {exc}")

    def _on_home_to_position(self) -> None:
        """Handle the home to position event."""
        axis = self._axis_combo.currentIndex()
        velocity = self._velocity_spin.value()
        axis_name = AXIS_NAMES[axis]
        offset = int(self._settings.get(
            self._axis_settings_key(axis_name, "offset_steps"), 0,
        ))
        direction = int(self._settings.get(
            self._axis_settings_key(axis_name, "home_direction"), 0,
        ))

        if offset == 0:
            self._log_message(
                f"Home {axis_name}",
                "No offset configured — set offset first via wizard",
            )
            return

        direction_label = _DIRECTION_LABELS[direction]
        
        try:
            ok, msg = self._client.set_home_offset(axis, direction, offset)
            if not ok:
                self._log_message(f"Home {axis_name}", f"Failed to set offset: {msg}")
                return
            
            self._log_message(
                f"Home {axis_name}",
                f"To position, vel={velocity:.3f} rad/s, offset={offset} steps from {direction_label}",
            )
            
            ok, msg = self._client.home_axis_to_position(
                axis, velocity, offset,
            )
            if ok:
                self._log_message(f"Home {axis_name}", "Started")
            else:
                self._log_message(f"Home {axis_name}", f"Failed: {msg}")
        except Exception as exc:
            self._log_message("Error", f"Home to position failed: {exc}")

    # ------------------------------------------------------------------ #
    # Offset wizard handlers                                               #
    # ------------------------------------------------------------------ #

    def _on_wiz_axis_changed(self) -> None:
        """Handle the wiz axis changed event."""
        axis = self._wiz_axis_combo.currentIndex()
        axis_name = AXIS_NAMES[axis]
        stored = self._settings.get(
            self._axis_settings_key(axis_name, "offset_steps"), 0,
        )
        stored_direction = int(self._settings.get(
            self._axis_settings_key(axis_name, "home_direction"), 0,
        ))
        direction_label = _DIRECTION_LABELS[stored_direction]
        self._wiz_stored_offset.setText(f"{stored} steps ({direction_label})")
        self._wiz_manual_offset.setValue(int(stored))
        self._wiz_direction_combo.setCurrentIndex(stored_direction)

    def _on_wiz_home_endstop(self) -> None:
        """Handle the wiz home endstop event."""
        axis = self._wiz_axis_combo.currentIndex()
        direction = self._wiz_direction_combo.currentIndex()
        velocity = self._wiz_velocity_spin.value()
        axis_name = AXIS_NAMES[axis]

        self._log_message(
            f"Wizard {axis_name}",
            f"Homing to endstop {_DIRECTION_LABELS[direction]}, "
            f"vel={velocity:.3f} rad/s",
        )
        try:
            ok, msg = self._client.home_axis_to_endstop(
                axis, direction, velocity,
            )
            if ok:
                self._log_message(f"Wizard {axis_name}", "Endstop homing started")
            else:
                self._log_message(f"Wizard {axis_name}", f"Failed: {msg}")
        except Exception as exc:
            self._log_message("Error", f"Wizard home to endstop failed: {exc}")

    def _on_wiz_reset_position(self) -> None:
        """Handle the reset position event."""
        axis = self._wiz_axis_combo.currentIndex()
        axis_name = AXIS_NAMES[axis]

        self._log_message(
            f"Wizard {axis_name}",
            "Resetting position counter to 0",
        )
        try:
            ok, msg = self._client.set_axis_position(axis, 0.0)
            if ok:
                self._log_message(f"Wizard {axis_name}", "Position reset to 0")
            else:
                self._log_message(f"Wizard {axis_name}", f"Failed: {msg}")
        except Exception as exc:
            self._log_message("Error", f"Reset position failed: {exc}")

    def _on_jog_minus(self) -> None:
        """Handle the jog minus event."""
        self._jog_axis(-1)

    def _on_jog_plus(self) -> None:
        """Handle the jog plus event."""
        self._jog_axis(1)

    def _jog_axis(self, sign: int) -> None:
        """Jog the axis."""
        axis = self._wiz_axis_combo.currentIndex()
        axis_name = AXIS_NAMES[axis]
        distance = self._wiz_jog_distance.value() * sign
        velocity = self._wiz_jog_velocity.value()

        self._log_message(
            f"Jog {axis_name}", f"{distance:+d} steps, vel={velocity:.3f} rad/s",
        )
        try:
            ok, msg = self._client.jog_axis(axis, distance, velocity)
            if ok:
                self._log_message(f"Jog {axis_name}", "Started")
            else:
                self._log_message(f"Jog {axis_name}", f"Failed: {msg}")
        except Exception as exc:
            self._log_message("Error", f"Jog failed: {exc}")

    def _on_capture_offset(self) -> None:
        """Handle the capture offset event."""
        axis = self._wiz_axis_combo.currentIndex()
        axis_name = AXIS_NAMES[axis]
        try:
            steps = self._client.get_current_position_steps(axis)
            offset = int(round(steps))
            self._wiz_manual_offset.setValue(offset)
            self._log_message(
                f"Wizard {axis_name}",
                f"Captured position: {offset} steps",
            )
        except Exception as exc:
            self._log_message("Error", f"Capture offset failed: {exc}")

    def _on_save_offset(self) -> None:
        """Handle the save offset event."""
        axis = self._wiz_axis_combo.currentIndex()
        axis_name = AXIS_NAMES[axis]
        offset = self._wiz_manual_offset.value()
        direction = self._wiz_direction_combo.currentIndex()

        self._settings.set(
            self._axis_settings_key(axis_name, "offset_steps"),
            offset,
        )
        self._settings.set(
            self._axis_settings_key(axis_name, "home_direction"),
            direction,
        )
        self._settings.save()
        
        direction_label = _DIRECTION_LABELS[direction]
        self._log_message(
            f"Wizard {axis_name}",
            f"Saving offset to firmware: {offset} steps from {direction_label} endstop",
        )
        
        try:
            ok, msg = self._client.set_home_offset(axis, direction, offset)
            if ok:
                self._wiz_stored_offset.setText(f"{offset} steps ({direction_label})")
                self._log_message(
                    f"Offset {axis_name}",
                    f"Saved to config and firmware: {offset} steps from {direction_label}",
                )
            else:
                self._log_message(f"Offset {axis_name}", f"Failed to save to firmware: {msg}")
        except Exception as exc:
            self._log_message("Error", f"Save offset to firmware failed: {exc}")

    # ------------------------------------------------------------------ #
    # Full robot homing                                                    #
    # ------------------------------------------------------------------ #

    def _format_sequence_options(self) -> list[str]:
        """Format the sequence options."""
        default_seq = " → ".join(AXIS_NAMES[i] for i in _DEFAULT_HOMING_SEQUENCE)
        reverse_seq = " → ".join(
            AXIS_NAMES[i] for i in reversed(_DEFAULT_HOMING_SEQUENCE)
        )
        return [
            f"Default ({default_seq})",
            f"Reverse ({reverse_seq})",
        ]

    def _get_selected_sequence(self) -> list[int]:
        """Return the selected sequence."""
        idx = self._sequence_combo.currentIndex()
        if idx == 1:
            return list(reversed(_DEFAULT_HOMING_SEQUENCE))
        return list(_DEFAULT_HOMING_SEQUENCE)

    def _validate_offsets_configured(self, sequence: list[int]) -> list[str]:
        """Returns axis names from the sequence that have no offset configured."""
        missing = []
        for axis_idx in sequence:
            axis_name = AXIS_NAMES[axis_idx]
            key = self._axis_settings_key(axis_name, "offset_steps")
            offset = self._settings.get(key, None)
            if offset is None or offset == 0:
                missing.append(axis_name)
        return missing

    def _on_home_all(self) -> None:
        """Handle the home all event."""
        if self._home_all_running:
            self._log_message("Home All", "Already in progress")
            return

        self._homing_sequence = self._get_selected_sequence()
        velocity = self._full_velocity_spin.value()

        missing = self._validate_offsets_configured(self._homing_sequence)
        if missing:
            names = ", ".join(missing)
            self._log_message(
                "Home All",
                f"Aborted — offsets not configured for: {names}",
            )
            self._full_progress_label.setText(
                f"Missing offsets: {names}",
            )
            return

        seq_names = " → ".join(AXIS_NAMES[i] for i in self._homing_sequence)

        self._log_message("Home All", f"Starting sequence: {seq_names}")
        self._log_message("Home All", f"Velocity: {velocity:.3f} rad/s")

        self._home_all_running = True
        self._home_all_index = 0
        self._btn_home_all.setEnabled(False)

        self._start_next_home_all_axis(velocity)

    def _start_next_home_all_axis(self, velocity: float) -> None:
        """Start the next home all axis."""
        if self._home_all_index >= len(self._homing_sequence):
            self._home_all_running = False
            self._btn_home_all.setEnabled(self._connected)
            self._full_progress_label.setText("Complete")
            self._log_message("Home All", "Sequence complete")
            return

        axis = self._homing_sequence[self._home_all_index]
        axis_name = AXIS_NAMES[axis]
        offset = int(self._settings.get(
            self._axis_settings_key(axis_name, "offset_steps"), 0,
        ))
        progress = f"{self._home_all_index + 1}/{len(self._homing_sequence)}"
        self._full_progress_label.setText(
            f"Homing {axis_name} ({progress})...",
        )

        try:
            ok, msg = self._client.home_axis_to_position(
                axis, velocity, offset,
            )
            if ok:
                self._log_message(
                    "Home All", f"Axis {axis_name} started ({progress})",
                )
            else:
                self._log_message(
                    "Home All", f"Axis {axis_name} failed: {msg}",
                )
                self._home_all_running = False
                self._btn_home_all.setEnabled(self._connected)
                self._full_progress_label.setText(f"Error on {axis_name}")
        except Exception as exc:
            self._log_message("Error", f"Home all failed: {exc}")
            self._home_all_running = False
            self._btn_home_all.setEnabled(self._connected)
            self._full_progress_label.setText("Error")

    def _check_home_all_progress(self, status: HomingStatus) -> None:
        """Check the home all progress."""
        if not self._home_all_running:
            return

        if status.any_active:
            return

        self._home_all_index += 1
        velocity = self._full_velocity_spin.value()
        self._start_next_home_all_axis(velocity)

    # ------------------------------------------------------------------ #
    # Emergency stop                                                       #
    # ------------------------------------------------------------------ #

    def _on_stop_all(self) -> None:
        """Handle the stop all event."""
        self._log_message("STOP", "Stopping all homing")
        self._home_all_running = False
        self._btn_home_all.setEnabled(self._connected)
        self._full_progress_label.setText("Stopped")

        try:
            ok, msg = self._client.stop_homing()
            if ok:
                self._log_message("STOP", "All homing stopped")
            else:
                self._log_message("STOP", f"Stop failed: {msg}")
        except Exception as exc:
            self._log_message("Error", f"Stop failed: {exc}")

    # ------------------------------------------------------------------ #
    # Log helpers                                                          #
    # ------------------------------------------------------------------ #

    def _log_message(self, source: str, message: str) -> None:
        """Log the message."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log_text.append(f"[{timestamp}] {source}: {message}")
        scrollbar = self._log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _clear_log(self) -> None:
        """Clear the log."""
        self._log_text.clear()
        self._log_message("System", "Log cleared")
