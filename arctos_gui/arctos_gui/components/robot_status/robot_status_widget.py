"""Robot status widget for the Arctos GUI.

Displays comprehensive robot status including diagnostics, joint states, and endstops.
Polls the RobotStatusClient at a fixed interval.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ...backend.app_state import AXIS_NAMES
from ...ui.theme import CARD_SPACING, OUTER_MARGIN, WIDGET_SPACING, set_role, set_status
from ...ui.widgets import action_button, connect as qt_connect, status_indicator_row
from .robot_status_client_protocol import (
    ConnectionData,
    DiagnosticData,
    EndstopAxisData,
    JointStateData,
    RobotStatusClient,
    STM32_CONNECTED,
    STM32_DISCONNECTED,
    STM32_RECONNECTING,
)

_POLL_INTERVAL_MS = 100
_STATUS_OK = "connected"
_STATUS_TRIGGERED = "disconnected"


@dataclass
class _EndstopRowWidgets:
    """Holds the Qt widgets for one endstop direction row."""

    dot: QLabel
    text: QLabel
    count_label: QLabel


@dataclass
class _JointRowWidgets:
    """Holds the Qt widgets for one joint state row."""

    homed_dot: QLabel
    pos_rad: QLabel
    vel_rad: QLabel
    pos_steps: QLabel


class RobotStatusWidget(QWidget):
    """Displays live robot status: diagnostics, joint states, and endstops.

    Uses a :class:`RobotStatusClient` to stay independent from ROS specifics.
    """

    def __init__(
        self,
        client: Optional[RobotStatusClient],
        parent: Optional[QWidget] = None,
    ) -> None:
        """Initializes the widget.

        Args:
            client: Robot status client. May be None when unavailable.
            parent: Optional parent widget.
        """
        super().__init__(parent)
        self._client = client

        self._build_ui()
        self._setup_poll_timer()

    def _build_ui(self) -> None:
        """Build the ui."""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN)
        main_layout.setSpacing(CARD_SPACING)

        # Top connection bar (ROS data source)
        conn_bar = QWidget()
        conn_layout = QHBoxLayout(conn_bar)
        conn_layout.setContentsMargins(0, 0, 0, 0)
        
        self._conn_dot, self._conn_text = status_indicator_row("No data source")
        conn_layout.addWidget(self._conn_dot)
        conn_layout.addWidget(self._conn_text)
        conn_layout.addStretch()
        
        self._last_update_label = QLabel("Last update: —")
        set_role(self._last_update_label, "muted")
        conn_layout.addWidget(self._last_update_label)
        
        main_layout.addWidget(conn_bar)

        # Main scrollable content
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)

        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(CARD_SPACING)

        # 2-column layout
        columns_layout = QHBoxLayout()
        columns_layout.setSpacing(CARD_SPACING)

        # Left column: Connection, Diagnostics & Trajectory
        left_col = QVBoxLayout()
        left_col.setSpacing(CARD_SPACING)
        left_col.addWidget(self._build_stm32_connection_card())
        left_col.addWidget(self._build_diagnostics_card())
        left_col.addWidget(self._build_trajectory_card())
        left_col.addStretch()

        # Right column: Joint States & Endstops
        right_col = QVBoxLayout()
        right_col.setSpacing(CARD_SPACING)
        right_col.addWidget(self._build_joint_states_card())
        right_col.addWidget(self._build_endstops_card())
        right_col.addStretch()

        columns_layout.addLayout(left_col, stretch=1)
        columns_layout.addLayout(right_col, stretch=2)

        scroll_layout.addLayout(columns_layout)
        scroll_area.setWidget(scroll_widget)
        main_layout.addWidget(scroll_area)

    def _build_stm32_connection_card(self) -> QFrame:
        """Build the STM32 connection status card."""
        card = QFrame()
        set_role(card, "card")
        layout = QVBoxLayout(card)

        title = QLabel("STM32 Connection")
        set_role(title, "title")
        layout.addWidget(title)

        status_row = QHBoxLayout()
        self._stm32_dot, self._stm32_text = status_indicator_row("Disconnected")
        status_row.addWidget(self._stm32_dot)
        status_row.addWidget(self._stm32_text)
        status_row.addStretch()
        layout.addLayout(status_row)

        form = QFormLayout()
        form.setSpacing(WIDGET_SPACING)

        self._stm32_last_recv_label = QLabel("—")
        self._stm32_attempts_label = QLabel("0")
        self._stm32_message_label = QLabel("—")
        set_role(self._stm32_message_label, "muted")

        last_recv_title = QLabel("Last received")
        set_role(last_recv_title, "fieldLabel")
        attempts_title = QLabel("Reconnect attempts")
        set_role(attempts_title, "fieldLabel")
        message_title = QLabel("Status")
        set_role(message_title, "fieldLabel")

        form.addRow(last_recv_title, self._stm32_last_recv_label)
        form.addRow(attempts_title, self._stm32_attempts_label)
        form.addRow(message_title, self._stm32_message_label)
        layout.addLayout(form)

        self._reconnect_btn = action_button("Reconnect", "warning")
        qt_connect(self._reconnect_btn.clicked, self._on_reconnect_clicked)
        layout.addWidget(self._reconnect_btn)

        return card

    def _build_diagnostics_card(self) -> QFrame:
        """Build the diagnostics card."""
        card = QFrame()
        set_role(card, "card")
        layout = QVBoxLayout(card)
        
        title = QLabel("System Diagnostics")
        set_role(title, "title")
        layout.addWidget(title)

        form = QFormLayout()
        form.setSpacing(WIDGET_SPACING)
        
        self._diag_labels = {
            "System State": QLabel("—"),
            "Uptime": QLabel("—"),
            "Homing Active": QLabel("—"),
            "Servo Pulse": QLabel("—"),
            "Broadcast Seq": QLabel("—"),
        }
        
        for key, label in self._diag_labels.items():
            field_label = QLabel(key)
            set_role(field_label, "fieldLabel")
            form.addRow(field_label, label)
            
        layout.addLayout(form)
        return card

    def _build_trajectory_card(self) -> QFrame:
        """Build the trajectory card."""
        card = QFrame()
        set_role(card, "card")
        layout = QVBoxLayout(card)
        
        title = QLabel("Trajectory Execution")
        set_role(title, "title")
        layout.addWidget(title)

        form = QFormLayout()
        form.setSpacing(WIDGET_SPACING)
        
        self._traj_id_label = QLabel("—")
        self._traj_points_label = QLabel("—")
        
        id_title = QLabel("Trajectory ID")
        set_role(id_title, "fieldLabel")
        points_title = QLabel("Points Loaded")
        set_role(points_title, "fieldLabel")
        
        form.addRow(id_title, self._traj_id_label)
        form.addRow(points_title, self._traj_points_label)
        layout.addLayout(form)
        
        self._traj_progress = QProgressBar()
        self._traj_progress.setRange(0, 100)
        self._traj_progress.setValue(0)
        self._traj_progress.setTextVisible(True)
        self._traj_progress.setFormat("Progress: %v% (Idle)")
        layout.addWidget(self._traj_progress)
        
        return card

    def _build_joint_states_card(self) -> QFrame:
        """Build the joint states card."""
        card = QFrame()
        set_role(card, "card")
        layout = QVBoxLayout(card)
        
        title = QLabel("Joint States")
        set_role(title, "title")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(WIDGET_SPACING)
        
        # Headers
        headers = ["Axis", "Homed", "Pos [rad]", "Vel [rad/s]", "Pos [steps]"]
        for col, text in enumerate(headers):
            lbl = QLabel(text)
            set_role(lbl, "fieldLabel")
            grid.addWidget(lbl, 0, col)

        self._joint_rows: dict[str, _JointRowWidgets] = {}
        
        for row, axis in enumerate(AXIS_NAMES, start=1):
            axis_lbl = QLabel(axis)
            set_role(axis_lbl, "fieldLabel")
            grid.addWidget(axis_lbl, row, 0)
            
            homed_dot = QLabel("●")
            set_role(homed_dot, "statusDot")
            set_status(homed_dot, "disconnected")
            
            pos_rad = QLabel("0.000")
            vel_rad = QLabel("0.000")
            pos_steps = QLabel("0")
            
            grid.addWidget(homed_dot, row, 1)
            grid.addWidget(pos_rad, row, 2)
            grid.addWidget(vel_rad, row, 3)
            grid.addWidget(pos_steps, row, 4)
            
            self._joint_rows[axis] = _JointRowWidgets(homed_dot, pos_rad, vel_rad, pos_steps)

        layout.addLayout(grid)
        return card

    def _build_endstops_card(self) -> QFrame:
        """Build the endstops card."""
        card = QFrame()
        set_role(card, "card")
        layout = QVBoxLayout(card)
        
        title = QLabel("Endstop Triggers")
        set_role(title, "title")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(WIDGET_SPACING)

        self._endstop_rows: dict[str, dict[str, _EndstopRowWidgets]] = {}

        for row, axis in enumerate(AXIS_NAMES):
            self._endstop_rows[axis] = {}
            
            axis_lbl = QLabel(axis)
            set_role(axis_lbl, "fieldLabel")
            grid.addWidget(axis_lbl, row, 0)
            
            for col_idx, direction in enumerate(("MIN", "MAX")):
                dot = QLabel("●")
                set_role(dot, "statusDot")
                set_status(dot, _STATUS_OK)
                
                text = QLabel(f"{direction}: OK")
                set_role(text, "statusText")
                set_status(text, _STATUS_OK)

                count_label = QLabel("×0")
                set_role(count_label, "muted")

                # Inner layout for one endstop
                inner_widget = QWidget()
                inner_layout = QHBoxLayout(inner_widget)
                inner_layout.setContentsMargins(0, 0, 0, 0)
                inner_layout.setSpacing(4)
                inner_layout.addWidget(dot)
                inner_layout.addWidget(text)
                inner_layout.addStretch()
                inner_layout.addWidget(count_label)
                
                grid.addWidget(inner_widget, row, col_idx + 1)
                
                self._endstop_rows[axis][direction] = _EndstopRowWidgets(
                    dot=dot, text=text, count_label=count_label
                )

        layout.addLayout(grid)
        return card

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

        self._update_stm32_connection(self._client.get_connection_data())
        self._update_diagnostics(self._client.get_diagnostic_data())
        self._update_trajectory(self._client.get_diagnostic_data())
        self._update_joints(self._client.get_joint_data())
        self._update_endstops(self._client.get_endstop_data())
        
        self._last_update_label.setText(
            f"Last update: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}"
        )

    def _update_connection_ui(self, connected: bool) -> None:
        """Update the ROS data source connection indicator."""
        if connected:
            set_status(self._conn_dot, "connected")
            set_status(self._conn_text, "connected")
            self._conn_text.setText("ROS Connected")
        else:
            set_status(self._conn_dot, "disconnected")
            set_status(self._conn_text, "disconnected")
            self._conn_text.setText("No data source")

    def _update_stm32_connection(self, data: ConnectionData) -> None:
        """Update the STM32 connection card."""
        if data.status == STM32_CONNECTED:
            set_status(self._stm32_dot, "connected")
            set_status(self._stm32_text, "connected")
            self._stm32_text.setText("Connected")
        elif data.status == STM32_RECONNECTING:
            set_status(self._stm32_dot, "warning")
            set_status(self._stm32_text, "warning")
            self._stm32_text.setText("Reconnecting...")
        else:
            set_status(self._stm32_dot, "disconnected")
            set_status(self._stm32_text, "disconnected")
            self._stm32_text.setText("Disconnected")

        if data.last_received_time > 0:
            elapsed = time.time() - data.last_received_time
            self._stm32_last_recv_label.setText(f"{elapsed:.1f}s ago")
        else:
            self._stm32_last_recv_label.setText("—")

        self._stm32_attempts_label.setText(str(data.reconnect_attempts))
        self._stm32_message_label.setText(data.message)

    def _on_reconnect_clicked(self) -> None:
        """Handle the reconnect button click."""
        if self._client is None:
            return
        self._reconnect_btn.setEnabled(False)
        self._reconnect_btn.setText("Reconnecting...")
        success, message = self._client.request_reconnect()
        self._reconnect_btn.setEnabled(True)
        self._reconnect_btn.setText("Reconnect")

    def _update_diagnostics(self, data: DiagnosticData) -> None:
        """Update the diagnostics."""
        self._diag_labels["System State"].setText(data.system_state)
        
        if data.system_state in ("ERROR", "STOPPING"):
            set_role(self._diag_labels["System State"], "danger")
        elif data.system_state in ("MOVING", "TRAJ_RUNNING"):
            set_role(self._diag_labels["System State"], "accent")
        else:
            set_role(self._diag_labels["System State"], "")

        self._diag_labels["Uptime"].setText(f"{data.uptime_s:.1f} s")
        self._diag_labels["Homing Active"].setText("Yes" if data.homing_active else "No")
        self._diag_labels["Servo Pulse"].setText(f"{data.servo_pulse_us} µs")
        self._diag_labels["Broadcast Seq"].setText(str(data.broadcast_seq))

    def _update_trajectory(self, data: DiagnosticData) -> None:
        """Update the trajectory."""
        self._traj_id_label.setText(str(data.trajectory_id))
        self._traj_points_label.setText(str(data.traj_points_loaded))
        
        if data.traj_total_segments > 0:
            progress = int((data.traj_current_segment / data.traj_total_segments) * 100)
            self._traj_progress.setMaximum(100)
            self._traj_progress.setValue(progress)
            
            if data.system_state == "TRAJ_RUNNING":
                self._traj_progress.setFormat(f"Segment {data.traj_current_segment}/{data.traj_total_segments} (%v%)")
            else:
                self._traj_progress.setFormat(f"Loaded {data.traj_total_segments} segments (%v%)")
        else:
            self._traj_progress.setValue(0)
            self._traj_progress.setFormat("No trajectory loaded")

    def _update_joints(self, data: dict[str, JointStateData]) -> None:
        """Update the joints."""
        for axis, state in data.items():
            if axis not in self._joint_rows:
                continue
                
            row = self._joint_rows[axis]
            
            set_status(row.homed_dot, "connected" if state.is_homed else "disconnected")
            row.homed_dot.setToolTip("Referenced" if state.is_homed else "Not Referenced")
            
            row.pos_rad.setText(f"{state.position_rad:8.3f}")
            row.vel_rad.setText(f"{state.velocity_rad_s:8.3f}")
            row.pos_steps.setText(f"{int(state.position_steps)}")

    def _update_endstops(self, data: dict[str, EndstopAxisData]) -> None:
        """Update the endstops."""
        for axis, axis_data in data.items():
            if axis not in self._endstop_rows:
                continue
            self._update_endstop_direction(
                self._endstop_rows[axis]["MIN"],
                axis,
                "MIN",
                axis_data.min_triggered,
                axis_data.min_trigger_count,
            )
            self._update_endstop_direction(
                self._endstop_rows[axis]["MAX"],
                axis,
                "MAX",
                axis_data.max_triggered,
                axis_data.max_trigger_count,
            )

    def _update_endstop_direction(
        self,
        row: _EndstopRowWidgets,
        axis: str,
        direction: str,
        triggered: bool,
        trigger_count: int,
    ) -> None:
        """Update the endstop direction."""
        status = _STATUS_TRIGGERED if triggered else _STATUS_OK
        label = f"{direction}: TRIGGERED" if triggered else f"{direction}: OK"
        set_status(row.dot, status)
        set_status(row.text, status)
        row.text.setText(label)
        row.count_label.setText(f"×{trigger_count}")
