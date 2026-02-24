"""Servo jog widget — real-time streaming jog via MoveIt Servo.

Provides two sub-tabs (Joint Mode / Cartesian Mode) with press-and-hold
jog buttons that publish velocity commands at ~30 Hz while held.
"""

from __future__ import annotations

from datetime import datetime
from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ...ui.theme import (
    CARD_SPACING,
    OUTER_MARGIN,
    WIDGET_SPACING,
    set_role,
    set_status,
    set_variant,
)
from ...ui.widgets import action_button
from ...ui.widgets import connect as qt_connect
from ...ui.widgets import field_label
from .servo_jog_client_protocol import ServoJogClient

JOINT_NAMES: List[str] = [
    "X_joint", "Y_joint", "Z_joint",
    "A_joint", "B_joint", "C_joint",
]
JOINT_DISPLAY_LABELS: List[str] = ["X", "Y", "Z", "A", "B", "C"]

PUBLISH_INTERVAL_MS = 33
STATUS_POLL_MS = 1000
POSE_POLL_MS = 200

SPEED_SLIDER_MIN = 5
SPEED_SLIDER_MAX = 100
SPEED_SLIDER_DEFAULT = 50
SPEED_SLIDER_TICK = 5

_ZERO_LINEAR: Tuple[float, float, float] = (0.0, 0.0, 0.0)
_ZERO_ANGULAR: Tuple[float, float, float] = (0.0, 0.0, 0.0)

SINGULARITY_THRESHOLD_RAD = 0.05


class ServoJogWidget(QWidget):
    """Real-time servo jog widget with joint and cartesian sub-tabs.

    Uses a :class:`ServoJogClient` protocol to stay independent from ROS.
    """

    def __init__(
        self,
        client: ServoJogClient,
        parent: Optional[QWidget] = None,
    ) -> None:
        """Initialize the widget.

        Args:
            client: Backend implementing the servo jog protocol.
            parent: Optional parent widget.
        """
        super().__init__(parent)
        self._client = client
        self._connected = False

        self._active_joint_jogs: Dict[str, float] = {}
        self._active_cartesian_linear: List[float] = [0.0, 0.0, 0.0]
        self._active_cartesian_angular: List[float] = [0.0, 0.0, 0.0]

        self._build_ui()
        self._setup_timers()

        self._log_message("System", "Servo jog widget initialized")
        self._update_status()

        self.setFocusPolicy(Qt.StrongFocus)

    # ------------------------------------------------------------------ #
    # UI construction                                                     #
    # ------------------------------------------------------------------ #

    def _build_ui(self) -> None:
        """Create the scrollable layout with connection, sub-tabs, and log."""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(
            OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN,
        )
        main_layout.setSpacing(CARD_SPACING)

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
        scroll_layout.setSpacing(CARD_SPACING)

        scroll_layout.addWidget(self._build_connection_group())
        scroll_layout.addWidget(self._build_singularity_warning())
        scroll_layout.addWidget(self._build_subtabs())
        scroll_layout.addWidget(self._build_log_group())
        scroll_layout.addStretch()

        scroll_area.setWidget(scroll_widget)
        main_layout.addWidget(scroll_area)

    def _build_connection_group(self) -> QGroupBox:
        """Construct the connection status card."""
        group = QGroupBox("Connection")
        layout = QVBoxLayout(group)
        layout.setSpacing(WIDGET_SPACING)

        row = QHBoxLayout()
        row.setSpacing(WIDGET_SPACING + 2)

        self._status_indicator = QLabel("\u25cf")
        set_role(self._status_indicator, "statusDot")
        set_status(self._status_indicator, "disconnected")

        self._status_text = QLabel("Disconnected")
        set_role(self._status_text, "statusText")
        set_status(self._status_text, "disconnected")

        self._connect_button = action_button("Connect", variant="primary")
        qt_connect(self._connect_button.clicked, self._toggle_connection)

        self._stop_button = action_button("STOP", variant="danger")
        self._stop_button.setEnabled(False)
        qt_connect(self._stop_button.clicked, self._emergency_stop)

        row.addWidget(self._status_indicator)
        row.addWidget(self._status_text)
        row.addStretch(1)
        row.addWidget(self._stop_button)
        row.addWidget(self._connect_button)
        layout.addLayout(row)

        info_row = QHBoxLayout()
        info_row.setSpacing(WIDGET_SPACING + 2)

        self._endpoint_label = QLabel(
            "MoveIt Servo \u2192 arctos_servo_controller \u2192 STM32"
        )
        set_role(self._endpoint_label, "muted")

        self._last_update = QLabel("Last update: -")
        set_role(self._last_update, "muted")

        info_row.addWidget(self._endpoint_label)
        info_row.addStretch(1)
        info_row.addWidget(self._last_update)
        layout.addLayout(info_row)

        return group

    def _build_singularity_warning(self) -> QLabel:
        """Construct the singularity warning banner (hidden by default)."""
        self._singularity_warning = QLabel(
            "\u26a0  Robot is near the all-zeros singularity. "
            "Cartesian jog will be very slow or unresponsive. "
            "Use Joint Mode to move at least one joint away from 0 first."
        )
        self._singularity_warning.setWordWrap(True)
        set_role(self._singularity_warning, "warning")
        self._singularity_warning.setVisible(False)
        return self._singularity_warning

    def _build_subtabs(self) -> QTabWidget:
        """Create the Joint / Cartesian sub-tab widget."""
        self._subtabs = QTabWidget()
        self._subtabs.setObjectName("subTabs")

        self._subtabs.addTab(self._build_joint_tab(), "Joint Mode")
        self._subtabs.addTab(self._build_cartesian_tab(), "Cartesian Mode")

        return self._subtabs

    # -- Joint mode tab ------------------------------------------------ #

    def _build_joint_tab(self) -> QWidget:
        """Construct the joint jog sub-tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, WIDGET_SPACING, 0, 0)
        layout.setSpacing(CARD_SPACING)

        layout.addWidget(self._build_joint_speed_group())
        layout.addWidget(self._build_joint_controls_group())
        layout.addStretch()
        return tab

    def _build_joint_speed_group(self) -> QGroupBox:
        """Construct the joint speed slider card."""
        group = QGroupBox("Joint Speed")
        layout = QHBoxLayout(group)
        layout.setSpacing(CARD_SPACING)

        label = field_label("Speed Scale:")
        self._joint_speed_slider = QSlider(Qt.Horizontal)
        self._joint_speed_slider.setRange(SPEED_SLIDER_MIN, SPEED_SLIDER_MAX)
        self._joint_speed_slider.setValue(SPEED_SLIDER_DEFAULT)
        self._joint_speed_slider.setTickInterval(SPEED_SLIDER_TICK)
        self._joint_speed_slider.setTickPosition(QSlider.TicksBelow)

        self._joint_speed_label = QLabel(
            f"{SPEED_SLIDER_DEFAULT / SPEED_SLIDER_MAX:.2f}"
        )
        self._joint_speed_label.setMinimumWidth(40)
        qt_connect(
            self._joint_speed_slider.valueChanged,
            self._on_joint_speed_changed,
        )

        layout.addWidget(label)
        layout.addWidget(self._joint_speed_slider, 1)
        layout.addWidget(self._joint_speed_label)
        return group

    def _build_joint_controls_group(self) -> QGroupBox:
        """Construct the per-joint jog button grid."""
        group = QGroupBox("Joint Jog Controls")
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(WIDGET_SPACING)
        grid.setVerticalSpacing(WIDGET_SPACING)

        header_joint = field_label("Joint")
        header_pos = field_label("Position [rad]")
        header_controls = field_label("Jog")
        grid.addWidget(header_joint, 0, 0)
        grid.addWidget(header_pos, 0, 1)
        grid.addWidget(header_controls, 0, 2, 1, 2)

        self._joint_pos_labels: List[QLabel] = []
        self._joint_buttons_minus: List[object] = []
        self._joint_buttons_plus: List[object] = []

        for i, (name, display) in enumerate(
            zip(JOINT_NAMES, JOINT_DISPLAY_LABELS)
        ):
            row = i + 1

            name_label = field_label(f"{display} ({name})")
            grid.addWidget(name_label, row, 0)

            pos_label = QLabel("-")
            pos_label.setSizePolicy(
                QSizePolicy.Expanding, QSizePolicy.Preferred,
            )
            self._joint_pos_labels.append(pos_label)
            grid.addWidget(pos_label, row, 1)

            btn_minus = action_button(f"\u2190 -{display}", variant="primary")
            btn_minus.setEnabled(False)
            btn_minus.installEventFilter(self)
            btn_minus.setProperty("jog_joint", name)
            btn_minus.setProperty("jog_direction", -1.0)
            self._joint_buttons_minus.append(btn_minus)
            grid.addWidget(btn_minus, row, 2)

            btn_plus = action_button(f"+{display} \u2192", variant="primary")
            btn_plus.setEnabled(False)
            btn_plus.installEventFilter(self)
            btn_plus.setProperty("jog_joint", name)
            btn_plus.setProperty("jog_direction", 1.0)
            self._joint_buttons_plus.append(btn_plus)
            grid.addWidget(btn_plus, row, 3)

        return group

    # -- Cartesian mode tab -------------------------------------------- #

    def _build_cartesian_tab(self) -> QWidget:
        """Construct the cartesian jog sub-tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, WIDGET_SPACING, 0, 0)
        layout.setSpacing(CARD_SPACING)

        layout.addWidget(self._build_cartesian_speed_group())
        layout.addWidget(self._build_pose_group())
        layout.addWidget(self._build_cartesian_controls_group())
        layout.addStretch()
        return tab

    def _build_cartesian_speed_group(self) -> QGroupBox:
        """Construct the cartesian speed slider card."""
        group = QGroupBox("Cartesian Speed")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(CARD_SPACING)
        layout.setVerticalSpacing(WIDGET_SPACING)

        lin_label = field_label("Linear:")
        self._cart_linear_slider = QSlider(Qt.Horizontal)
        self._cart_linear_slider.setRange(SPEED_SLIDER_MIN, SPEED_SLIDER_MAX)
        self._cart_linear_slider.setValue(SPEED_SLIDER_DEFAULT)
        self._cart_linear_slider.setTickInterval(SPEED_SLIDER_TICK)
        self._cart_linear_slider.setTickPosition(QSlider.TicksBelow)
        self._cart_linear_label = QLabel(
            f"{SPEED_SLIDER_DEFAULT / SPEED_SLIDER_MAX:.2f}"
        )
        self._cart_linear_label.setMinimumWidth(40)
        qt_connect(
            self._cart_linear_slider.valueChanged,
            self._on_cart_linear_speed_changed,
        )

        rot_label = field_label("Rotational:")
        self._cart_rot_slider = QSlider(Qt.Horizontal)
        self._cart_rot_slider.setRange(SPEED_SLIDER_MIN, SPEED_SLIDER_MAX)
        self._cart_rot_slider.setValue(SPEED_SLIDER_DEFAULT)
        self._cart_rot_slider.setTickInterval(SPEED_SLIDER_TICK)
        self._cart_rot_slider.setTickPosition(QSlider.TicksBelow)
        self._cart_rot_label = QLabel(
            f"{SPEED_SLIDER_DEFAULT / SPEED_SLIDER_MAX:.2f}"
        )
        self._cart_rot_label.setMinimumWidth(40)
        qt_connect(
            self._cart_rot_slider.valueChanged,
            self._on_cart_rot_speed_changed,
        )

        layout.addWidget(lin_label, 0, 0)
        layout.addWidget(self._cart_linear_slider, 0, 1)
        layout.addWidget(self._cart_linear_label, 0, 2)
        layout.addWidget(rot_label, 1, 0)
        layout.addWidget(self._cart_rot_slider, 1, 1)
        layout.addWidget(self._cart_rot_label, 1, 2)

        return group

    def _build_pose_group(self) -> QGroupBox:
        """Construct the live end-effector pose display."""
        group = QGroupBox("End-Effector Pose")
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(CARD_SPACING)
        grid.setVerticalSpacing(WIDGET_SPACING)

        self._pose_labels: Dict[str, QLabel] = {}
        axes = ["X", "Y", "Z", "RX", "RY", "RZ"]
        for i, axis in enumerate(axes):
            label = field_label(f"{axis}:")
            value = QLabel("-")
            value.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            self._pose_labels[axis.lower()] = value

            row = i // 3
            col = (i % 3) * 2
            grid.addWidget(label, row, col)
            grid.addWidget(value, row, col + 1)

        return group

    def _build_cartesian_controls_group(self) -> QGroupBox:
        """Construct translation and rotation jog button groups."""
        group = QGroupBox("Cartesian Jog Controls")
        layout = QVBoxLayout(group)
        layout.setSpacing(CARD_SPACING)

        trans_group = QGroupBox("Translation")
        trans_grid = QGridLayout(trans_group)
        trans_grid.setHorizontalSpacing(WIDGET_SPACING)
        trans_grid.setVerticalSpacing(WIDGET_SPACING)

        cart_axes_trans = [
            ("X", 0, "linear"),
            ("Y", 1, "linear"),
            ("Z", 2, "linear"),
        ]
        self._cart_buttons: List[object] = []

        for axis_label, axis_idx, axis_type in cart_axes_trans:
            row = axis_idx
            btn_minus = action_button(
                f"\u2190 -{axis_label}", variant="primary",
            )
            btn_minus.setEnabled(False)
            btn_minus.installEventFilter(self)
            btn_minus.setProperty("cart_type", axis_type)
            btn_minus.setProperty("cart_index", axis_idx)
            btn_minus.setProperty("cart_direction", -1.0)
            self._cart_buttons.append(btn_minus)
            trans_grid.addWidget(btn_minus, row, 0)

            btn_plus = action_button(
                f"+{axis_label} \u2192", variant="primary",
            )
            btn_plus.setEnabled(False)
            btn_plus.installEventFilter(self)
            btn_plus.setProperty("cart_type", axis_type)
            btn_plus.setProperty("cart_index", axis_idx)
            btn_plus.setProperty("cart_direction", 1.0)
            self._cart_buttons.append(btn_plus)
            trans_grid.addWidget(btn_plus, row, 1)

        layout.addWidget(trans_group)

        rot_group = QGroupBox("Rotation")
        rot_grid = QGridLayout(rot_group)
        rot_grid.setHorizontalSpacing(WIDGET_SPACING)
        rot_grid.setVerticalSpacing(WIDGET_SPACING)

        cart_axes_rot = [
            ("RX", 0, "angular"),
            ("RY", 1, "angular"),
            ("RZ", 2, "angular"),
        ]

        for axis_label, axis_idx, axis_type in cart_axes_rot:
            row = axis_idx
            btn_minus = action_button(
                f"\u2190 -{axis_label}", variant="primary",
            )
            btn_minus.setEnabled(False)
            btn_minus.installEventFilter(self)
            btn_minus.setProperty("cart_type", axis_type)
            btn_minus.setProperty("cart_index", axis_idx)
            btn_minus.setProperty("cart_direction", -1.0)
            self._cart_buttons.append(btn_minus)
            rot_grid.addWidget(btn_minus, row, 0)

            btn_plus = action_button(
                f"+{axis_label} \u2192", variant="primary",
            )
            btn_plus.setEnabled(False)
            btn_plus.installEventFilter(self)
            btn_plus.setProperty("cart_type", axis_type)
            btn_plus.setProperty("cart_index", axis_idx)
            btn_plus.setProperty("cart_direction", 1.0)
            self._cart_buttons.append(btn_plus)
            rot_grid.addWidget(btn_plus, row, 1)

        layout.addWidget(rot_group)
        return group

    def _build_log_group(self) -> QGroupBox:
        """Construct the system log panel."""
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
        self._log_text.setMaximumHeight(180)
        set_role(self._log_text, "log")
        layout.addWidget(self._log_text)

        return group

    # ------------------------------------------------------------------ #
    # Timers                                                              #
    # ------------------------------------------------------------------ #

    def _setup_timers(self) -> None:
        """Start periodic timers for status, pose, and command publishing."""
        self._status_timer = QTimer()
        qt_connect(self._status_timer.timeout, self._update_status)
        self._status_timer.start(STATUS_POLL_MS)

        self._pose_timer = QTimer()
        qt_connect(self._pose_timer.timeout, self._update_pose)
        self._pose_timer.start(POSE_POLL_MS)

        self._publish_timer = QTimer()
        qt_connect(self._publish_timer.timeout, self._publish_commands)
        self._publish_timer.start(PUBLISH_INTERVAL_MS)

    # ------------------------------------------------------------------ #
    # Press-and-hold via eventFilter                                      #
    # ------------------------------------------------------------------ #

    def eventFilter(  # noqa: N802 (Qt override)
        self, obj: object, event: QEvent,
    ) -> bool:
        """Intercept button press/release for continuous jog."""
        event_type = event.type()

        if event_type == QEvent.MouseButtonPress:
            self._handle_button_press(obj)
            return False

        if event_type == QEvent.MouseButtonRelease:
            self._handle_button_release(obj)
            return False

        return super().eventFilter(obj, event)

    def _handle_button_press(self, btn: object) -> None:
        """Start continuous jog for the pressed button."""
        joint_name = btn.property("jog_joint")
        if joint_name is not None:
            direction = float(btn.property("jog_direction"))
            speed = self._joint_speed_slider.value() / SPEED_SLIDER_MAX
            self._active_joint_jogs[joint_name] = direction * speed
            return

        cart_type = btn.property("cart_type")
        if cart_type is not None:
            idx = int(btn.property("cart_index"))
            direction = float(btn.property("cart_direction"))
            if cart_type == "linear":
                speed = (
                    self._cart_linear_slider.value() / SPEED_SLIDER_MAX
                )
                self._active_cartesian_linear[idx] = direction * speed
            else:
                speed = self._cart_rot_slider.value() / SPEED_SLIDER_MAX
                self._active_cartesian_angular[idx] = direction * speed

    def _handle_button_release(self, btn: object) -> None:
        """Stop jog for the released button."""
        joint_name = btn.property("jog_joint")
        if joint_name is not None:
            self._active_joint_jogs.pop(joint_name, None)
            if not self._active_joint_jogs and self._connected:
                self._client.stop()
            return

        cart_type = btn.property("cart_type")
        if cart_type is not None:
            idx = int(btn.property("cart_index"))
            if cart_type == "linear":
                self._active_cartesian_linear[idx] = 0.0
            else:
                self._active_cartesian_angular[idx] = 0.0

            all_zero = (
                all(v == 0.0 for v in self._active_cartesian_linear)
                and all(v == 0.0 for v in self._active_cartesian_angular)
            )
            if all_zero and self._connected:
                self._client.stop()

    # ------------------------------------------------------------------ #
    # Keyboard input                                                      #
    # ------------------------------------------------------------------ #

    def keyPressEvent(self, event) -> None:  # noqa: N802 (Qt override)
        """Handle keyboard shortcuts for jog control."""
        if event.isAutoRepeat():
            return

        key = event.key()

        if key == Qt.Key_Escape:
            self._emergency_stop()
            return

        if key == Qt.Key_Space:
            self._emergency_stop()
            return

        if not self._connected:
            super().keyPressEvent(event)
            return

        is_joint_tab = self._subtabs.currentIndex() == 0

        if is_joint_tab:
            self._handle_joint_key_press(key)
        else:
            self._handle_cartesian_key_press(key)

    def keyReleaseEvent(self, event) -> None:  # noqa: N802 (Qt override)
        """Handle key release to stop jog."""
        if event.isAutoRepeat():
            return

        key = event.key()

        is_joint_tab = self._subtabs.currentIndex() == 0

        if is_joint_tab:
            if key in (Qt.Key_W, Qt.Key_S):
                self._active_joint_jogs.clear()
                if self._connected:
                    self._client.stop()
        else:
            self._handle_cartesian_key_release(key)

    def _handle_joint_key_press(self, key: int) -> None:
        """Process joint-mode key presses."""
        joint_key_map = {
            Qt.Key_1: 0, Qt.Key_2: 1, Qt.Key_3: 2,
            Qt.Key_4: 3, Qt.Key_5: 4, Qt.Key_6: 5,
        }

        if key in joint_key_map:
            idx = joint_key_map[key]
            self._keyboard_selected_joint = idx
            self._log_message(
                "Keyboard",
                f"Selected joint: {JOINT_DISPLAY_LABELS[idx]}",
            )
            return

        selected = getattr(self, "_keyboard_selected_joint", 0)
        speed = self._joint_speed_slider.value() / SPEED_SLIDER_MAX
        name = JOINT_NAMES[selected]

        if key == Qt.Key_W:
            self._active_joint_jogs[name] = speed
        elif key == Qt.Key_S:
            self._active_joint_jogs[name] = -speed

    def _handle_cartesian_key_press(self, key: int) -> None:
        """Process cartesian-mode key presses."""
        lin_speed = self._cart_linear_slider.value() / SPEED_SLIDER_MAX
        rot_speed = self._cart_rot_slider.value() / SPEED_SLIDER_MAX

        key_map = {
            Qt.Key_W: ("linear", 0, lin_speed),
            Qt.Key_S: ("linear", 0, -lin_speed),
            Qt.Key_A: ("linear", 1, lin_speed),
            Qt.Key_D: ("linear", 1, -lin_speed),
            Qt.Key_Q: ("linear", 2, lin_speed),
            Qt.Key_E: ("linear", 2, -lin_speed),
            Qt.Key_I: ("angular", 0, rot_speed),
            Qt.Key_K: ("angular", 0, -rot_speed),
            Qt.Key_J: ("angular", 1, rot_speed),
            Qt.Key_L: ("angular", 1, -rot_speed),
            Qt.Key_U: ("angular", 2, rot_speed),
            Qt.Key_O: ("angular", 2, -rot_speed),
        }

        if key in key_map:
            axis_type, idx, value = key_map[key]
            if axis_type == "linear":
                self._active_cartesian_linear[idx] = value
            else:
                self._active_cartesian_angular[idx] = value

    def _handle_cartesian_key_release(self, key: int) -> None:
        """Process cartesian-mode key releases."""
        release_map = {
            Qt.Key_W: ("linear", 0),
            Qt.Key_S: ("linear", 0),
            Qt.Key_A: ("linear", 1),
            Qt.Key_D: ("linear", 1),
            Qt.Key_Q: ("linear", 2),
            Qt.Key_E: ("linear", 2),
            Qt.Key_I: ("angular", 0),
            Qt.Key_K: ("angular", 0),
            Qt.Key_J: ("angular", 1),
            Qt.Key_L: ("angular", 1),
            Qt.Key_U: ("angular", 2),
            Qt.Key_O: ("angular", 2),
        }

        if key in release_map:
            axis_type, idx = release_map[key]
            if axis_type == "linear":
                self._active_cartesian_linear[idx] = 0.0
            else:
                self._active_cartesian_angular[idx] = 0.0

            all_zero = (
                all(v == 0.0 for v in self._active_cartesian_linear)
                and all(v == 0.0 for v in self._active_cartesian_angular)
            )
            if all_zero and self._connected:
                self._client.stop()

    # ------------------------------------------------------------------ #
    # Command publishing (30 Hz timer)                                    #
    # ------------------------------------------------------------------ #

    def _publish_commands(self) -> None:
        """Publish active jog commands at the timer rate."""
        if not self._connected:
            return

        is_joint_tab = self._subtabs.currentIndex() == 0

        if is_joint_tab and self._active_joint_jogs:
            names = list(self._active_joint_jogs.keys())
            velocities = list(self._active_joint_jogs.values())
            try:
                self._client.multi_joint_jog(names, velocities)
            except Exception:  # noqa: BLE001
                pass
            return

        has_cartesian = (
            any(v != 0.0 for v in self._active_cartesian_linear)
            or any(v != 0.0 for v in self._active_cartesian_angular)
        )
        if not is_joint_tab and has_cartesian:
            linear = (
                self._active_cartesian_linear[0],
                self._active_cartesian_linear[1],
                self._active_cartesian_linear[2],
            )
            angular = (
                self._active_cartesian_angular[0],
                self._active_cartesian_angular[1],
                self._active_cartesian_angular[2],
            )
            try:
                self._client.cartesian_jog(linear=linear, angular=angular)
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------ #
    # Status & pose updates                                               #
    # ------------------------------------------------------------------ #

    def _update_status(self) -> None:
        """Refresh connection indicators."""
        is_connected = bool(self._client and self._client.is_connected())
        if is_connected != self._connected:
            self._set_connection_ui(is_connected)
            if is_connected:
                self._log_message("Connection", "Connected to MoveIt Servo")
            else:
                self._log_message("Connection", "Disconnected")
            self._update_joint_positions()
            self._update_pose()

        if is_connected:
            self._last_update.setText(
                f"Last update: {datetime.now().strftime('%H:%M:%S')}"
            )
            self._update_joint_positions()
            self._check_singularity()
        else:
            if self._last_update.text() != "Last update: -":
                self._last_update.setText("Last update: -")
            self._singularity_warning.setVisible(False)

    def _update_joint_positions(self) -> None:
        """Refresh joint position labels."""
        if not self._connected:
            for label in self._joint_pos_labels:
                if label.text() != "-":
                    label.setText("-")
            return

        try:
            positions = self._client.get_joint_positions()
            for i, name in enumerate(JOINT_NAMES):
                pos = positions.get(name, 0.0)
                self._joint_pos_labels[i].setText(f"{pos:.4f}")
        except Exception:  # noqa: BLE001
            pass

    def _update_pose(self) -> None:
        """Query the backend for the latest EE pose."""
        if not self._connected:
            for label in self._pose_labels.values():
                if label.text() != "-":
                    label.setText("-")
            return

        try:
            pose = self._client.get_ee_pose()
            self._pose_labels["x"].setText(f"{pose['x'] * 1000:.1f} mm")
            self._pose_labels["y"].setText(f"{pose['y'] * 1000:.1f} mm")
            self._pose_labels["z"].setText(f"{pose['z'] * 1000:.1f} mm")
            self._pose_labels["rx"].setText(f"{pose['rx']:.3f} rad")
            self._pose_labels["ry"].setText(f"{pose['ry']:.3f} rad")
            self._pose_labels["rz"].setText(f"{pose['rz']:.3f} rad")
        except Exception:  # noqa: BLE001
            for label in self._pose_labels.values():
                if label.text() != "-":
                    label.setText("-")

    def _check_singularity(self) -> None:
        """Show a warning if all joints are near the all-zeros singularity."""
        try:
            positions = self._client.get_joint_positions()
            near_zero = all(
                abs(positions.get(name, 0.0)) < SINGULARITY_THRESHOLD_RAD
                for name in JOINT_NAMES
            )
            self._singularity_warning.setVisible(near_zero)
        except Exception:  # noqa: BLE001
            self._singularity_warning.setVisible(False)

    def _set_connection_ui(self, connected: bool) -> None:
        """Toggle UI controls based on connection state."""
        self._connected = connected

        if connected:
            self._status_text.setText("Connected")
            set_status(self._status_indicator, "connected")
            set_status(self._status_text, "connected")
            self._connect_button.setText("Disconnect")
            set_variant(self._connect_button, "danger")
            self._stop_button.setEnabled(True)
        else:
            self._status_text.setText("Disconnected")
            set_status(self._status_indicator, "disconnected")
            set_status(self._status_text, "disconnected")
            self._connect_button.setText("Connect")
            set_variant(self._connect_button, "primary")
            self._stop_button.setEnabled(False)

        for btn in self._joint_buttons_minus:
            btn.setEnabled(connected)
        for btn in self._joint_buttons_plus:
            btn.setEnabled(connected)
        for btn in self._cart_buttons:
            btn.setEnabled(connected)

    # ------------------------------------------------------------------ #
    # Slider callbacks                                                    #
    # ------------------------------------------------------------------ #

    def _on_joint_speed_changed(self, value: int) -> None:
        """Update the joint speed display label."""
        self._joint_speed_label.setText(f"{value / SPEED_SLIDER_MAX:.2f}")

    def _on_cart_linear_speed_changed(self, value: int) -> None:
        """Update the cartesian linear speed display label."""
        self._cart_linear_label.setText(f"{value / SPEED_SLIDER_MAX:.2f}")

    def _on_cart_rot_speed_changed(self, value: int) -> None:
        """Update the cartesian rotational speed display label."""
        self._cart_rot_label.setText(f"{value / SPEED_SLIDER_MAX:.2f}")

    # ------------------------------------------------------------------ #
    # Actions                                                             #
    # ------------------------------------------------------------------ #

    def _toggle_connection(self) -> None:
        """Connect or disconnect depending on the current state."""
        if self._connected:
            self._log_message("Connection", "Disconnecting...")
            try:
                self._client.disconnect()
            finally:
                self._update_status()
            return

        self._log_message("Connection", "Connecting to MoveIt Servo...")
        try:
            ok = self._client.connect()
        except Exception as exc:  # noqa: BLE001
            ok = False
            self._log_message("Error", f"Connect failed: {exc}")

        if not ok:
            self._log_message("Error", "Connect failed")

        self._update_status()

    def _emergency_stop(self) -> None:
        """Immediately stop all motion."""
        self._active_joint_jogs.clear()
        self._active_cartesian_linear = [0.0, 0.0, 0.0]
        self._active_cartesian_angular = [0.0, 0.0, 0.0]

        if self._connected:
            try:
                self._client.stop()
            except Exception:  # noqa: BLE001
                pass
        self._log_message("STOP", "Emergency stop triggered")

    # ------------------------------------------------------------------ #
    # Logging                                                             #
    # ------------------------------------------------------------------ #

    def _log_message(self, source: str, message: str) -> None:
        """Append a timestamped message to the local log."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log_text.append(f"[{timestamp}] {source}: {message}")
        scrollbar = self._log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _clear_log(self) -> None:
        """Clear the log pane."""
        self._log_text.clear()
        self._log_message("System", "Log cleared")
