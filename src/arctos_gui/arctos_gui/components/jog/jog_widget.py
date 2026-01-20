"""Jog widget for Arctos GUI with modern UI design."""

from typing import Optional, Protocol
from datetime import datetime

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
    QTextEdit,
    QFrame,
    QGridLayout,
    QScrollArea,
    QGroupBox,
)


class JogClient(Protocol):
    """Abstraction for something that can execute jog commands.

    Keeps the Qt layer independent from ROS specifics.
    """

    def send_jog(self, axis_index: int, delta_rad: float, velocity_rad_s: float) -> None:
        ...

    def connect(self) -> bool:
        ...

    def disconnect(self) -> None:
        ...

    def is_connected(self) -> bool:
        ...


class JogWidget(QWidget):
    """Jog control widget for the Arctos arm with modern UI design.

    Features:
    - Modern professional UI with status indicators
    - Real-time connection status display
    - Integrated log/debug window
    - Jog controls with visual feedback
    """

    AXIS_LABELS = ["X", "Y", "Z", "A", "B", "C"]

    def __init__(self, client: JogClient, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._client = client
        self._connected = False
        self._build_ui()
        self._setup_status_timer()
        self._log_message("System", "Jog widget initialized")

    def _build_ui(self) -> None:
        # Main layout with scroll area for responsiveness
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # Create scroll area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Scroll widget
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(15)

        # Status Bar Group
        status_group = QGroupBox("Connection Status")
        status_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 13px;
                color: #333333;
                border: 2px solid #e0e0e0;
                border-radius: 8px;
                margin-top: 6px;
                padding-top: 10px;
                background-color: #fafafa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        status_layout = QVBoxLayout(status_group)
        
        # Connection status row
        conn_row = QHBoxLayout()
        
        # Status indicator
        self._status_indicator = QLabel("●")
        self._status_indicator.setStyleSheet("""
            QLabel {
                font-size: 24px;
                color: #d83b01;
                padding: 0 5px;
            }
        """)
        
        # Status text
        self._status_text = QLabel("Disconnected")
        self._status_text.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: #d83b01;
            }
        """)
        
        # Connect/Disconnect button
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
            QPushButton:pressed {
                background-color: #0e5a9a;
            }
        """)
        
        conn_row.addWidget(self._status_indicator)
        conn_row.addWidget(self._status_text)
        conn_row.addStretch()
        conn_row.addWidget(self._connect_button)
        status_layout.addLayout(conn_row)
        
        # Connection info
        info_row = QHBoxLayout()
        self._host_label = QLabel("Host: 192.168.178.159:8888")
        self._host_label.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: #666666;
            }
        """)
        self._last_update = QLabel("Last update: Never")
        self._last_update.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: #666666;
            }
        """)
        info_row.addWidget(self._host_label)
        info_row.addStretch()
        info_row.addWidget(self._last_update)
        status_layout.addLayout(info_row)
        
        scroll_layout.addWidget(status_group)

        # Jog Controls Group
        jog_group = QGroupBox("Jog Controls")
        jog_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 13px;
                color: #333333;
                border: 2px solid #e0e0e0;
                border-radius: 8px;
                margin-top: 6px;
                padding-top: 10px;
                background-color: #fafafa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        jog_layout = QGridLayout(jog_group)
        jog_layout.setSpacing(10)

        # Axis selection
        axis_label = QLabel("Axis:")
        axis_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                color: #555555;
                font-size: 13px;
            }
        """)
        self._axis_combo = QComboBox()
        self._axis_combo.addItems(self.AXIS_LABELS)
        self._axis_combo.setStyleSheet("""
            QComboBox {
                padding: 6px;
                border: 2px solid #e0e0e0;
                border-radius: 6px;
                background-color: white;
                min-width: 120px;
                font-size: 13px;
            }
            QComboBox:hover {
                border-color: #0078d4;
            }
            QComboBox:focus {
                border-color: #0078d4;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #666666;
                margin-right: 5px;
            }
        """)
        jog_layout.addWidget(axis_label, 0, 0)
        jog_layout.addWidget(self._axis_combo, 0, 1)

        # Delta
        delta_label = QLabel("Delta [rad]:")
        delta_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                color: #555555;
                font-size: 13px;
            }
        """)
        self._delta_spin = QDoubleSpinBox()
        self._delta_spin.setDecimals(4)
        self._delta_spin.setRange(-6.283, 6.283)  # +/- 2*pi
        self._delta_spin.setSingleStep(0.1)
        self._delta_spin.setValue(0.1)
        self._delta_spin.setStyleSheet("""
            QDoubleSpinBox {
                padding: 6px;
                border: 2px solid #e0e0e0;
                border-radius: 6px;
                background-color: white;
                min-width: 120px;
                font-size: 13px;
            }
            QDoubleSpinBox:hover {
                border-color: #0078d4;
            }
            QDoubleSpinBox:focus {
                border-color: #0078d4;
            }
        """)
        jog_layout.addWidget(delta_label, 1, 0)
        jog_layout.addWidget(self._delta_spin, 1, 1)

        # Velocity
        vel_label = QLabel("Velocity [rad/s]:")
        vel_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                color: #555555;
                font-size: 13px;
            }
        """)
        self._velocity_spin = QDoubleSpinBox()
        self._velocity_spin.setDecimals(3)
        self._velocity_spin.setRange(0.01, 10.0)
        self._velocity_spin.setSingleStep(0.1)
        self._velocity_spin.setValue(0.5)
        self._velocity_spin.setStyleSheet("""
            QDoubleSpinBox {
                padding: 6px;
                border: 2px solid #e0e0e0;
                border-radius: 6px;
                background-color: white;
                min-width: 120px;
                font-size: 13px;
            }
            QDoubleSpinBox:hover {
                border-color: #0078d4;
            }
            QDoubleSpinBox:focus {
                border-color: #0078d4;
            }
        """)
        jog_layout.addWidget(vel_label, 2, 0)
        jog_layout.addWidget(self._velocity_spin, 2, 1)

        # Jog buttons
        button_layout = QHBoxLayout()
        self._btn_minus = QPushButton("←  Jog Negative")
        self._btn_plus = QPushButton("Jog Positive  →")
        self._btn_minus.clicked.connect(self._on_jog_minus)
        self._btn_plus.clicked.connect(self._on_jog_plus)
        
        button_style = """
            QPushButton {
                background-color: #0078d4;
                color: white;
                border: none;
                padding: 12px 24px;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
                min-width: 140px;
            }
            QPushButton:hover {
                background-color: #106ebe;
            }
            QPushButton:pressed {
                background-color: #0e5a9a;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """
        
        self._btn_minus.setStyleSheet(button_style)
        self._btn_plus.setStyleSheet(button_style)
        self._btn_minus.setEnabled(False)
        self._btn_plus.setEnabled(False)
        
        button_layout.addStretch()
        button_layout.addWidget(self._btn_minus)
        button_layout.addSpacing(20)
        button_layout.addWidget(self._btn_plus)
        button_layout.addStretch()

        jog_layout.addLayout(button_layout, 3, 0, 1, 2)
        scroll_layout.addWidget(jog_group)

        # Log/Debug Window Group
        log_group = QGroupBox("System Log")
        log_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 13px;
                color: #333333;
                border: 2px solid #e0e0e0;
                border-radius: 8px;
                margin-top: 6px;
                padding-top: 10px;
                background-color: #fafafa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        log_layout = QVBoxLayout(log_group)
        
        # Log controls
        log_controls = QHBoxLayout()
        self._log_clear = QPushButton("Clear")
        self._log_clear.clicked.connect(self._clear_log)
        self._log_clear.setStyleSheet("""
            QPushButton {
                background-color: #f3f2f1;
                color: #323130;
                border: 1px solid #d2d0ce;
                padding: 4px 12px;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #edebe9;
            }
        """)
        log_controls.addStretch()
        log_controls.addWidget(self._log_clear)
        log_layout.addLayout(log_controls)
        
        # Log text area
        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumHeight(200)
        self._log_text.setStyleSheet("""
            QTextEdit {
                background-color: #ffffff;
                border: 1px solid #d2d0ce;
                border-radius: 4px;
                padding: 5px;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 11px;
                color: #323130;
            }
        """)
        log_layout.addWidget(self._log_text)
        
        scroll_layout.addWidget(log_group)
        scroll_layout.addStretch()

        # Set scroll widget
        scroll_area.setWidget(scroll_widget)
        main_layout.addWidget(scroll_area)

    def _setup_status_timer(self) -> None:
        """Setup timer for periodic status updates."""
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(1000)  # Update every second

    def _update_status(self) -> None:
        """Update connection status display."""
        if self._client:
            was_connected = self._connected
            self._connected = self._client.is_connected()
            
            # Update status if changed
            if was_connected != self._connected:
                if self._connected:
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
                        QPushButton:pressed {
                            background-color: #841f06;
                        }
                    """)
                    self._btn_minus.setEnabled(True)
                    self._btn_plus.setEnabled(True)
                    self._log_message("Connection", "Successfully connected to STM32")
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
                        QPushButton:pressed {
                            background-color: #0e5a9a;
                        }
                    """)
                    self._btn_minus.setEnabled(False)
                    self._btn_plus.setEnabled(False)
                    self._log_message("Connection", "Connection lost")

            # Update last update time
            if self._connected:
                self._last_update.setText(f"Last update: {datetime.now().strftime('%H:%M:%S')}")

    def _log_message(self, source: str, message: str) -> None:
        """Add a message to the log window."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {source}: {message}"
        self._log_text.append(log_entry)
        
        # Auto-scroll to bottom
        scrollbar = self._log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _clear_log(self) -> None:
        """Clear the log window."""
        self._log_text.clear()
        self._log_message("System", "Log cleared")

    def _current_axis_index(self) -> int:
        index = self._axis_combo.currentIndex()
        return int(index)

    def _current_delta(self) -> float:
        return float(self._delta_spin.value())

    def _current_velocity(self) -> float:
        return float(self._velocity_spin.value())

    def _on_jog_plus(self) -> None:
        self._send_jog(+1.0)

    def _on_jog_minus(self) -> None:
        self._send_jog(-1.0)

    def _send_jog(self, direction: float) -> None:
        if not self._connected or not self._client:
            self._log_message("Error", "Not connected to STM32")
            return
            
        axis_index = self._current_axis_index()
        axis_name = self.AXIS_LABELS[axis_index]
        delta = self._current_delta() * direction
        velocity = self._current_velocity()

        if velocity <= 0.0:
            velocity = 0.5

        self._log_message(f"Jog {axis_name}", 
                         f"Delta={delta:.4f} rad, Velocity={velocity:.3f} rad/s")
        
        try:
            self._client.send_jog(axis_index, delta, velocity)
            self._log_message(f"Jog {axis_name}", "Command sent successfully")
        except Exception as e:
            self._log_message("Error", f"Failed to send jog command: {str(e)}")
        
    def _toggle_connection(self) -> None:
        """Toggle Ethernet connection."""
        if not self._client:
            return
            
        if self._connected:
            self._log_message("Connection", "Disconnecting from STM32...")
            self._client.disconnect()
            self._connected = False
            self._update_status()  # Force immediate update
        else:
            self._log_message("Connection", "Connecting to STM32...")
            if self._client.connect():
                self._connected = True
                self._update_status()  # Force immediate update
            else:
                self._log_message("Error", "Failed to connect to STM32")
