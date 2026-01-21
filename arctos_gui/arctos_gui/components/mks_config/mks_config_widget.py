"""Refactored MKS configuration widget with clean architecture."""

from typing import Optional, Dict
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, 
    QPushButton, QTextEdit, QScrollArea, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QObject
from PyQt5.QtGui import QFont, QPalette, QColor

from .mks_can_direct import MKSCanDirect
from .base_config_tab import BaseConfigTab
from .advanced_config_tab import AdvancedConfigTab
from .homing_tab import HomingTab
from .motor_control_tab import MotorControlTab
from .data_read_tab import DataReadTab


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
    
    def execute_operation(self, op_type: str, **kwargs):
        """Execute an operation in the worker thread."""
        if op_type in self._operations:
            try:
                result = self._operations[op_type](**kwargs)
                self.operation_completed.emit(op_type, result)
            except Exception as e:
                self.operation_completed.emit(op_type, {"success": False, "message": f"Error: {str(e)}"})
    
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
        self.worker.operation_completed.connect(self._on_operation_completed)
        self.worker_thread.start()
        
        self._connected = False
        
        # Initialize UI
        self.init_ui()
        self.apply_modern_style()
        
        # Setup connection status timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_connection_status)
        self.update_timer.start(1000)
    
    def init_ui(self) -> None:
        """Initialize the user interface layout and components."""
        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Title and status bar
        header_layout = self._create_header()
        main_layout.addLayout(header_layout)
        
        # Tab widget
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #cccccc;
                background-color: white;
            }
            QTabBar::tab {
                background-color: #f0f0f0;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: white;
                border-bottom: 2px solid #0078d4;
            }
            QTabBar::tab:hover {
                background-color: #e5e5e5;
            }
        """)
        
        # Create tabs
        self._create_tabs()
        
        main_layout.addWidget(self.tab_widget)
        
        # Status log
        log_group = self._create_status_log()
        main_layout.addWidget(log_group)
        
        self.setLayout(main_layout)
    
    def _create_header(self) -> QHBoxLayout:
        """Create the header with title and connection status."""
        layout = QHBoxLayout()
        
        # Title
        title_label = QLabel("MKS Servo Motor Configuration")
        title_label.setStyleSheet("""
            QLabel {
                font-size: 18px;
                font-weight: bold;
                color: #333333;
            }
        """)
        layout.addWidget(title_label)
        
        layout.addStretch()
        
        # Connection status
        status_group = QGroupBox()
        status_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #cccccc;
                border-radius: 6px;
                padding: 5px;
                background-color: white;
            }
        """)
        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(10, 5, 10, 5)
        
        self._status_indicator = QLabel("●")
        self._status_indicator.setStyleSheet("""
            QLabel {
                font-size: 24px;
                color: #d83b01;
                padding: 0 5px;
            }
        """)
        status_layout.addWidget(self._status_indicator)
        
        self._status_text = QLabel("Disconnected")
        self._status_text.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: #d83b01;
            }
        """)
        status_layout.addWidget(self._status_text)
        
        self._connect_button = QPushButton("Connect")
        self._connect_button.clicked.connect(self._toggle_connection)
        self._connect_button.setStyleSheet("""
            QPushButton {
                background-color: #0078d4;
                color: white;
                border: none;
                padding: 8px 20px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 13px;
                min-width: 100px;
            }
            QPushButton:hover {
                background-color: #106ebe;
            }
        """)
        status_layout.addWidget(self._connect_button)
        
        self._last_update = QLabel("")
        self._last_update.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: #666666;
                padding-left: 10px;
            }
        """)
        status_layout.addWidget(self._last_update)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        return layout
    
    def _create_tabs(self) -> None:
        """Create all configuration tabs."""
        # Base Configuration Tab
        self.base_tab = BaseConfigTab()
        self.base_tab.operation_requested.connect(self._on_tab_operation)
        self.tab_widget.addTab(self.base_tab, "Basic Config")
        
        # Advanced Configuration Tab
        self.advanced_tab = AdvancedConfigTab()
        self.advanced_tab.operation_requested.connect(self._on_tab_operation)
        self.tab_widget.addTab(self.advanced_tab, "Advanced")
        
        # Homing Tab
        self.homing_tab = HomingTab()
        self.homing_tab.operation_requested.connect(self._on_tab_operation)
        self.tab_widget.addTab(self.homing_tab, "Homing")
        
        # Motor Control Tab
        self.motor_control_tab = MotorControlTab()
        self.motor_control_tab.operation_requested.connect(self._on_tab_operation)
        self.tab_widget.addTab(self.motor_control_tab, "Motor Control")
        
        # Data Read Tab
        self.data_read_tab = DataReadTab()
        self.data_read_tab.operation_requested.connect(self._on_tab_operation)
        self.tab_widget.addTab(self.data_read_tab, "Data Read")
    
    def _create_status_log(self) -> QGroupBox:
        """Create the status log display."""
        group = QGroupBox("Status Log")
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.status_log = QTextEdit()
        self.status_log.setMaximumHeight(100)
        self.status_log.setStyleSheet("""
            QTextEdit {
                font-family: 'Courier New', monospace;
                font-size: 11px;
                background-color: #f8f8f8;
                border: 1px solid #dddddd;
            }
        """)
        layout.addWidget(self.status_log)
        
        clear_btn = QPushButton("Clear Log")
        clear_btn.setMaximumWidth(100)
        clear_btn.clicked.connect(self.status_log.clear)
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
        
        # Execute operation in worker thread
        self.worker.execute_operation(operation, **params)
    
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
            self.worker.execute_operation('disconnect')
        else:
            self.log_status("Connecting to CAN interface...")
            self.worker.execute_operation('connect')
    
    def _update_connection_status(self):
        """Update connection status based on CAN client state."""
        if self.can_client:
            was_connected = self._connected
            self._connected = self.can_client.is_connected()
            
            if was_connected != self._connected:
                self._update_connection_display(self._connected, 
                    "Connected" if self._connected else "Disconnected")
    
    def _update_connection_display(self, connected: bool, message: str = ""):
        """Update connection status display."""
        self._connected = connected
        
        if connected:
            self._status_indicator.setStyleSheet("""
                QLabel {
                    font-size: 24px;
                    color: #107c10;
                    padding: 0 5px;
                }
            """)
            self._status_text.setText("Connected")
            self._status_text.setStyleSheet("""
                QLabel {
                    font-size: 14px;
                    font-weight: bold;
                    color: #107c10;
                }
            """)
            self._connect_button.setText("Disconnect")
            self._connect_button.setStyleSheet("""
                QPushButton {
                    background-color: #d83b01;
                    color: white;
                    border: none;
                    padding: 8px 20px;
                    border-radius: 6px;
                    font-weight: bold;
                    font-size: 13px;
                    min-width: 100px;
                }
                QPushButton:hover {
                    background-color: #a7260a;
                }
            """)
            self._last_update.setText(f"Last update: {self.get_timestamp()}")
            if message and message != "Connected":
                self.log_status(f"✓ Connected to CAN interface: {message}", True)
        else:
            self._status_indicator.setStyleSheet("""
                QLabel {
                    font-size: 24px;
                    color: #d83b01;
                    padding: 0 5px;
                }
            """)
            self._status_text.setText("Disconnected")
            self._status_text.setStyleSheet("""
                QLabel {
                    font-size: 14px;
                    font-weight: bold;
                    color: #d83b01;
                }
            """)
            self._connect_button.setText("Connect")
            self._connect_button.setStyleSheet("""
                QPushButton {
                    background-color: #0078d4;
                    color: white;
                    border: none;
                    padding: 8px 20px;
                    border-radius: 6px;
                    font-weight: bold;
                    font-size: 13px;
                    min-width: 100px;
                }
                QPushButton:hover {
                    background-color: #106ebe;
                }
            """)
            if message and message != "Disconnected":
                self.log_status(f"✗ Disconnected: {message}", False)
            else:
                self.log_status("✗ Disconnected from CAN interface", False)
    
    def log_status(self, message: str, success: bool = True) -> None:
        """Add a message to the status log.
        
        Args:
            message: Message to log
            success: Whether the operation was successful
        """
        timestamp = self.get_timestamp()
        icon = "✓" if success else "✗"
        color = "green" if success else "red"
        
        self.status_log.append(
            f'<span style="color: #666;">[{timestamp}]</span> '
            f'<span style="color: {color}; font-weight: bold;">{icon}</span> '
            f'{message}'
        )
        
        # Auto-scroll to bottom
        scrollbar = self.status_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def get_timestamp(self) -> str:
        """Get current timestamp as formatted string.
        
        Returns:
            str: Current time in HH:MM:SS format
        """
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S")
    
    def apply_modern_style(self) -> None:
        """Apply modern styling to the widget."""
        self.setStyleSheet("""
            QWidget {
                background-color: #f5f5f5;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e0e0e0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
    
    def closeEvent(self, event):
        """Clean up resources when widget is closed."""
        # Stop worker thread
        if self.worker_thread.isRunning():
            self.worker_thread.quit()
            self.worker_thread.wait(1000)
        
        # Disconnect CAN
        if self.can_client:
            self.can_client.disconnect()
        
        super().closeEvent(event)
