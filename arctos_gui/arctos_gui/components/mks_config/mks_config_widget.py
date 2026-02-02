"""Refactored MKS configuration widget with clean architecture."""

from __future__ import annotations

import html
from datetime import datetime
from typing import Any, Callable, Optional

from PyQt5.QtCore import QTimer, pyqtSignal, QThread, QObject
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import (
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    QStyle,
)

from ...ui.theme import set_role, set_status, set_variant
from ...ui.widgets import action_button
from ...ui.widgets import scroll_container

from .mks_can_direct import MKSCanDirect
from .base_config_tab import BaseConfigTab
from .advanced_config_tab import AdvancedConfigTab
from .homing_tab import HomingTab
from .motor_control_tab import MotorControlTab
from .data_read_tab import DataReadTab


def _connect(signal: Any, slot: Callable[..., object]) -> None:
    """Connect a Qt signal to a slot.

    Typing helper for environments without PyQt type stubs.
    """

    signal.connect(slot)  # type: ignore


class MKSWorker(QObject):
    """Worker thread for executing CAN operations without blocking GUI."""
    
    operation_completed = pyqtSignal(str, dict)  # operation_type, result
    
    def __init__(self, can_client: MKSCanDirect):
        super().__init__()
        self.can_client = can_client
        self._operations = {
            'connect': self._connect,
            'disconnect': self._disconnect,
            'calibrate': self._calibrate,
            'set_work_mode': self._set_work_mode,
            'set_working_current': self._set_working_current,
            'set_holding_current': self._set_holding_current,
            'set_subdivision': self._set_subdivision,
            'restore_defaults': self._restore_defaults,
            'set_home_params': self._set_home_params,
            'go_home': self._go_home,
            'set_zero': self._set_zero,
            'set_limit_remap': self._set_limit_remap,
            'enable_motor': self._enable_motor,
            'query_status': self._query_status,
            'read_encoder': self._read_encoder,
            'read_speed': self._read_speed,
            'read_io_status': self._read_io_status,
            'read_motor_status': self._read_motor_status,
        }
    
    def execute_operation(self, op_type: str, params: dict) -> None:
        """Execute an operation in the worker thread."""
        if op_type not in self._operations:
            return

        try:
            result = self._operations[op_type](**params)
            self.operation_completed.emit(op_type, result)
        except Exception as exc:  # noqa: BLE001
            self.operation_completed.emit(
                op_type,
                {"success": False, "message": f"Error: {exc}"},
            )
    
    # Connection operations
    def _connect(self):
        return {"success": self.can_client.connect(), "message": "Connected" if self.can_client.is_connected() else "Failed"}
    
    def _disconnect(self):
        self.can_client.disconnect()
        return {"success": True, "message": "Disconnected"}
    
    # Configuration operations
    def _calibrate(self, motor_id: int):
        return self.can_client.calibrate_motor(motor_id)
    
    def _set_work_mode(self, motor_id: int, work_mode: int):
        return self.can_client.set_work_mode(motor_id, work_mode)
    
    def _set_working_current(self, motor_id: int, current_ma: int):
        return self.can_client.set_working_current(motor_id, current_ma)
    
    def _set_holding_current(self, motor_id: int, percentage: int):
        return self.can_client.set_holding_current(motor_id, percentage)
    
    def _set_subdivision(self, motor_id: int, subdivision: int):
        return self.can_client.set_subdivision(motor_id, subdivision)
    
    def _restore_defaults(self, motor_id: int):
        return self.can_client.restore_defaults(motor_id)
    
    # Homing operations
    def _set_home_params(self, motor_id: int, trigger: int, direction: int, speed: int, enable_limit: bool):
        return self.can_client.set_home_parameters(motor_id, trigger, direction, speed, enable_limit)
    
    def _go_home(self, motor_id: int):
        return self.can_client.go_home(motor_id)
    
    def _set_zero(self, motor_id: int):
        return self.can_client.set_zero_position(motor_id)
    
    def _set_limit_remap(self, motor_id: int, enable: bool):
        return self.can_client.set_limit_remap(motor_id, enable)
    
    # Motor control operations
    def _enable_motor(self, motor_id: int, enable: bool):
        return self.can_client.enable_motor(motor_id, enable)
    
    def _query_status(self, motor_id: int):
        return self.can_client.query_status(motor_id)
    
    # Data reading operations
    def _read_encoder(self, motor_id: int):
        return self.can_client.read_encoder(motor_id)
    
    def _read_speed(self, motor_id: int):
        return self.can_client.read_speed(motor_id)
    
    def _read_io_status(self, motor_id: int):
        return self.can_client.read_io_status(motor_id)
    
    def _read_motor_status(self, motor_id: int):
        return self.can_client.read_motor_status(motor_id)


class MKSConfigWidget(QWidget):
    """Configuration widget for MKS servo motors.
    
    Clean architecture with separate tabs for different functional areas.
    """
    
    operation_requested = pyqtSignal(str, dict)
    status_updated = pyqtSignal(str, bool)
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        """Initialize the MKS configuration widget.
        
        Args:
            parent: Parent widget (optional)
        """
        super().__init__(parent)
        
        # Initialize CAN client and worker
        self.can_client = MKSCanDirect()
        self.worker_thread = QThread()
        self.worker = MKSWorker(self.can_client)
        self.worker.moveToThread(self.worker_thread)
        _connect(self.worker.operation_completed, self._on_operation_completed)
        _connect(self.operation_requested, self.worker.execute_operation)
        self.worker_thread.start()
        
        self._connected = False
        
        # Initialize UI
        self.init_ui()
        
        # Setup connection status timer
        self.update_timer = QTimer()
        _connect(self.update_timer.timeout, self._update_connection_status)
        self.update_timer.start(1000)
    
    def init_ui(self) -> None:
        """Initialize the user interface layout and components."""
        outer_layout = QVBoxLayout(self)
        outer_layout.setSpacing(12)
        outer_layout.setContentsMargins(0, 0, 0, 0)

        content = QWidget()
        main_layout = QVBoxLayout(content)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Title and status bar
        header_layout = self._create_header()
        main_layout.addLayout(header_layout)
        
        # Tab widget
        self.tab_widget = QTabWidget()
        self.tab_widget.setObjectName("subTabs")
        self.tab_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        _connect(self.tab_widget.currentChanged, self._sync_tab_height)
        
        # Create tabs
        self._create_tabs()

        self._sync_tab_height()
        
        main_layout.addWidget(self.tab_widget)
        
        # Status log
        log_group = self._create_status_log()
        main_layout.addWidget(log_group)

        outer_layout.addWidget(scroll_container(content))


    def _sync_tab_height(self) -> None:
        current = self.tab_widget.currentWidget() if hasattr(self, "tab_widget") else None
        if current is None:
            return

        content_h = int(current.sizeHint().height())
        bar_h = int(self.tab_widget.tabBar().sizeHint().height())
        margins = self.tab_widget.contentsMargins()
        frame = int(self.tab_widget.style().pixelMetric(QStyle.PM_DefaultFrameWidth)) * 2

        total = content_h + bar_h + int(margins.top() + margins.bottom()) + frame
        self.tab_widget.setFixedHeight(max(1, int(total)))

    
    def _create_header(self) -> QHBoxLayout:
        """Create the header with title and connection status."""
        layout = QHBoxLayout()
        
        # Title
        title_label = QLabel("MKS Servo Motor Configuration")
        set_role(title_label, "title")
        layout.addWidget(title_label)
        
        layout.addStretch()
        
        # Connection status
        status_group = QFrame()
        set_role(status_group, "card")
        status_group.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)

        status_outer = QVBoxLayout(status_group)
        status_outer.setContentsMargins(10, 8, 10, 8)
        status_outer.setSpacing(4)

        status_layout = QHBoxLayout()
        status_layout.setSpacing(10)
        
        self._status_indicator = QLabel("●")
        set_role(self._status_indicator, "statusDot")
        set_status(self._status_indicator, "disconnected")
        status_layout.addWidget(self._status_indicator)
        
        self._status_text = QLabel("Disconnected")
        set_role(self._status_text, "statusText")
        set_status(self._status_text, "disconnected")
        status_layout.addWidget(self._status_text)

        status_layout.addStretch(1)
        
        self._connect_button = action_button("Connect", variant="primary")
        self._connect_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        _connect(self._connect_button.clicked, self._toggle_connection)
        status_layout.addWidget(self._connect_button)

        status_outer.addLayout(status_layout)

        self._last_update = QLabel("")
        set_role(self._last_update, "muted")
        status_outer.addWidget(self._last_update)
        layout.addWidget(status_group)
        
        return layout
    
    def _create_tabs(self) -> None:
        """Create all configuration tabs."""
        # Base Configuration Tab
        self.base_tab = BaseConfigTab()
        _connect(self.base_tab.operation_requested, self._on_tab_operation)
        self.tab_widget.addTab(self.base_tab, "Basic Config")
        
        # Advanced Configuration Tab
        self.advanced_tab = AdvancedConfigTab()
        _connect(self.advanced_tab.operation_requested, self._on_tab_operation)
        self.tab_widget.addTab(self.advanced_tab, "Advanced")
        
        # Homing Tab
        self.homing_tab = HomingTab()
        _connect(self.homing_tab.operation_requested, self._on_tab_operation)
        self.tab_widget.addTab(self.homing_tab, "Homing")
        
        # Motor Control Tab
        self.motor_control_tab = MotorControlTab()
        _connect(self.motor_control_tab.operation_requested, self._on_tab_operation)
        self.tab_widget.addTab(self.motor_control_tab, "Motor Control")
        
        # Data Read Tab
        self.data_read_tab = DataReadTab()
        _connect(self.data_read_tab.operation_requested, self._on_tab_operation)
        self.tab_widget.addTab(self.data_read_tab, "Data Read")
    
    def _create_status_log(self) -> QGroupBox:
        """Create the status log display."""
        group = QGroupBox("Status Log")
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)
        
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(100)
        set_role(self.status_log, "log")
        layout.addWidget(self.status_log)
        
        clear_btn = QPushButton("Clear Log")
        clear_btn.setMaximumWidth(100)
        _connect(clear_btn.clicked, self.status_log.clear)
        layout.addWidget(clear_btn)
        
        group.setLayout(layout)
        return group
    
    def _on_tab_operation(self, operation: str, params: dict):
        """Handle operation request from a tab.
        
        Args:
            operation: Operation name
            params: Operation parameters
        """
        # Add motor_id if not present (use motor control tab's selection)
        if 'motor_id' not in params and hasattr(self, 'motor_control_tab'):
            params['motor_id'] = self.motor_control_tab.get_motor_id()
        
        self.operation_requested.emit(operation, params)
    
    def _on_operation_completed(self, op_type: str, result: dict):
        """Handle completion of a CAN operation.
        
        Args:
            op_type: Type of operation completed
            result: Operation result
        """
        success = result.get("success", False)
        message = result.get("message", "")
        
        # Update connection status
        if op_type == 'connect':
            self._update_connection_display(success, message)
        elif op_type == 'disconnect':
            self._update_connection_display(False, message)
        
        # Update data displays
        if op_type == 'read_encoder' and success:
            self.data_read_tab.update_encoder_display(result)
        elif op_type == 'read_speed' and success:
            self.data_read_tab.update_speed_display(result)
        elif op_type == 'read_io_status' and success:
            self.data_read_tab.update_io_display(result)
        elif op_type == 'read_motor_status' and success:
            self.data_read_tab.update_motor_status_display(result)
        
        # Log message
        self.log_status(message, success)
    
    def _toggle_connection(self):
        """Toggle CAN connection."""
        if self._connected:
            self.log_status("Disconnecting from CAN interface...")
            self.operation_requested.emit("disconnect", {})
        else:
            self.log_status("Connecting to CAN interface...")
            self.operation_requested.emit("connect", {})
    
    def _update_connection_status(self):
        """Update connection status based on CAN client state."""
        if self.can_client:
            was_connected = self._connected
            self._connected = self.can_client.is_connected()

            if self._connected:
                self._last_update.setText(f"Last update: {self.get_timestamp()}")
            
            if was_connected != self._connected:
                self._update_connection_display(
                    self._connected,
                    "Connected" if self._connected else "Disconnected",
                )
    
    def _update_connection_display(self, connected: bool, message: str = ""):
        """Update connection status display."""
        self._connected = connected

        if connected:
            self._status_text.setText("Connected")
            set_status(self._status_indicator, "connected")
            set_status(self._status_text, "connected")
            self._connect_button.setText("Disconnect")
            set_variant(self._connect_button, "danger")
            self._last_update.setText(f"Last update: {self.get_timestamp()}")
            if message and message != "Connected":
                self.log_status(f"Connected: {message}", True)
            return

        self._status_text.setText("Disconnected")
        set_status(self._status_indicator, "disconnected")
        set_status(self._status_text, "disconnected")
        self._connect_button.setText("Connect")
        set_variant(self._connect_button, "primary")
        self._last_update.setText("")
        if message and message != "Disconnected":
            self.log_status(f"Disconnected: {message}", False)
        else:
            self.log_status("Disconnected", False)
    
    def log_status(self, message: str, success: bool = True) -> None:
        """Add a message to the status log.
        
        Args:
            message: Message to log
            success: Whether the operation was successful
        """
        timestamp = self.get_timestamp()
        icon = "✓" if success else "✗"
        icon_color = "#22c55e" if success else "#ef4444"
        safe_message = html.escape(message)

        self.status_log.append(
            f'<span style="color: #9ca3af;">[{timestamp}]</span> '
            f'<span style="color: {icon_color}; font-weight: 700;">{icon}</span> '
            f'{safe_message}'
        )
        
        # Auto-scroll to bottom
        scrollbar = self.status_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def get_timestamp(self) -> str:
        """Get current timestamp as formatted string.
        
        Returns:
            str: Current time in HH:MM:SS format
        """
        return datetime.now().strftime("%H:%M:%S")

    def shutdown(self) -> None:
        if hasattr(self, "update_timer") and self.update_timer.isActive():
            self.update_timer.stop()

        if self.can_client:
            self.can_client.disconnect()

        if self.worker_thread.isRunning():
            self.worker_thread.requestInterruption()
            self.worker_thread.quit()
            self.worker_thread.wait(1000)

    def closeEvent(self, a0: QCloseEvent) -> None:  # noqa: N802 (Qt override)
        """Clean up resources when widget is closed."""
        self.shutdown()
        
        super().closeEvent(a0)
