"""Data reading tab for MKS servo motors."""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, 
    QPushButton, QComboBox, QFormLayout, QTextEdit
)
from PyQt5.QtCore import Qt, pyqtSignal


class DataReadTab(QWidget):
    """Data reading tab for MKS motors.
    
    Provides controls for reading encoder, speed, IO, and motor status.
    """
    
    operation_requested = pyqtSignal(str, dict)  # operation_name, parameters
    
    def __init__(self, parent=None):
        """Initialize the data reading tab.
        
        Args:
            parent: Parent widget
        """
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        """Initialize the user interface."""
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Motor Selection
        motor_select_group = QGroupBox("Motor Selection")
        motor_select_layout = QFormLayout()
        motor_select_layout.setSpacing(10)
        motor_select_layout.setLabelAlignment(Qt.AlignRight)
        
        self.motor_combo = QComboBox()
        self.motor_combo.addItems([
            "X - Joint 1",
            "Y - Joint 2", 
            "Z - Joint 3",
            "A - Joint 4",
            "B - Joint 5",
            "C - Joint 6"
        ])
        self.motor_combo.currentIndexChanged.connect(self._on_motor_changed)
        motor_select_layout.addRow("Select Motor:", self.motor_combo)
        
        self.motor_id_label = QLabel("Motor ID: 1")
        self.motor_id_label.setStyleSheet("font-weight: bold; color: #1976D2;")
        motor_select_layout.addRow("", self.motor_id_label)
        
        motor_select_group.setLayout(motor_select_layout)
        layout.addWidget(motor_select_group)
        
        # Read Controls
        read_group = QGroupBox("Data Read Operations")
        read_layout = QVBoxLayout()
        read_layout.setSpacing(10)
        
        # Read buttons in grid
        button_layout = QVBoxLayout()
        
        # First row - Position and Speed
        row1_layout = QHBoxLayout()
        
        encoder_btn = QPushButton("Read Encoder")
        encoder_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        encoder_btn.clicked.connect(self._on_read_encoder)
        row1_layout.addWidget(encoder_btn)
        
        speed_btn = QPushButton("Read Speed")
        speed_btn.setStyleSheet("""
            QPushButton {
                background-color: #00BCD4;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #00ACC1;
            }
        """)
        speed_btn.clicked.connect(self._on_read_speed)
        row1_layout.addWidget(speed_btn)
        
        button_layout.addLayout(row1_layout)
        
        # Second row - IO and Status
        row2_layout = QHBoxLayout()
        
        io_btn = QPushButton("Read I/O Status")
        io_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
        """)
        io_btn.clicked.connect(self._on_read_io_status)
        row2_layout.addWidget(io_btn)
        
        status_btn = QPushButton("Read Motor Status")
        status_btn.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
        """)
        status_btn.clicked.connect(self._on_read_motor_status)
        row2_layout.addWidget(status_btn)
        
        button_layout.addLayout(row2_layout)
        
        read_layout.addLayout(button_layout)
        
        # Read all button
        read_all_btn = QPushButton("Read All Data")
        read_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 12px;
                border-radius: 4px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        read_all_btn.clicked.connect(self._on_read_all)
        read_layout.addWidget(read_all_btn)
        
        read_group.setLayout(read_layout)
        layout.addWidget(read_group)
        
        # Data Display
        display_group = QGroupBox("Data Display")
        display_layout = QVBoxLayout()
        display_layout.setSpacing(5)
        
        # Create display widgets for different data types
        self.encoder_display = QTextEdit()
        self.encoder_display.setMaximumHeight(80)
        self.encoder_display.setHtml("<span style='color: #666;'>No encoder data</span>")
        display_layout.addWidget(QLabel("Encoder Position:"))
        display_layout.addWidget(self.encoder_display)
        
        self.speed_display = QTextEdit()
        self.speed_display.setMaximumHeight(60)
        self.speed_display.setHtml("<span style='color: #666;'>No speed data</span>")
        display_layout.addWidget(QLabel("Motor Speed:"))
        display_layout.addWidget(self.speed_display)
        
        self.io_display = QTextEdit()
        self.io_display.setMaximumHeight(100)
        self.io_display.setHtml("<span style='color: #666;'>No I/O data</span>")
        display_layout.addWidget(QLabel("I/O Status:"))
        display_layout.addWidget(self.io_display)
        
        self.motor_status_display = QTextEdit()
        self.motor_status_display.setMaximumHeight(120)
        self.motor_status_display.setHtml("<span style='color: #666;'>No motor status data</span>")
        display_layout.addWidget(QLabel("Motor Status:"))
        display_layout.addWidget(self.motor_status_display)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        layout.addStretch()
        self.setLayout(layout)
        
        # Initialize with first motor selected
        self._on_motor_changed(0)
    
    def _on_motor_changed(self, index):
        """Handle motor selection change.
        
        Args:
            index: Index of selected motor in combo box
        """
        motor_mapping = {
            0: 1,  # X - Joint 1
            1: 2,  # Y - Joint 2
            2: 3,  # Z - Joint 3
            3: 4,  # A - Joint 4
            4: 5,  # B - Joint 5
            5: 6   # C - Joint 6
        }
        
        motor_id = motor_mapping[index]
        self.motor_id_label.setText(f"Motor ID: {motor_id}")
        
        # Clear displays when motor changes
        self.clear_displays()
    
    def clear_displays(self):
        """Clear all data displays."""
        self.encoder_display.setHtml("<span style='color: #666;'>No encoder data</span>")
        self.speed_display.setHtml("<span style='color: #666;'>No speed data</span>")
        self.io_display.setHtml("<span style='color: #666;'>No I/O data</span>")
        self.motor_status_display.setHtml("<span style='color: #666;'>No motor status data</span>")
    
    def get_motor_id(self):
        """Get the currently selected motor ID.
        
        Returns:
            int: Motor ID (1-6)
        """
        motor_mapping = {
            0: 1,  # X - Joint 1
            1: 2,  # Y - Joint 2
            2: 3,  # Z - Joint 3
            3: 4,  # A - Joint 4
            4: 5,  # B - Joint 5
            5: 6   # C - Joint 6
        }
        
        return motor_mapping[self.motor_combo.currentIndex()]
    
    def _on_read_encoder(self):
        """Handle encoder reading button click."""
        motor_id = self.get_motor_id()
        self.operation_requested.emit('read_encoder', {'motor_id': motor_id})
    
    def _on_read_speed(self):
        """Handle speed reading button click."""
        motor_id = self.get_motor_id()
        self.operation_requested.emit('read_speed', {'motor_id': motor_id})
    
    def _on_read_io_status(self):
        """Handle I/O status reading button click."""
        motor_id = self.get_motor_id()
        self.operation_requested.emit('read_io_status', {'motor_id': motor_id})
    
    def _on_read_motor_status(self):
        """Handle motor status reading button click."""
        motor_id = self.get_motor_id()
        self.operation_requested.emit('read_motor_status', {'motor_id': motor_id})
    
    def _on_read_all(self):
        """Handle read all data button click."""
        motor_id = self.get_motor_id()
        # Clear displays first
        self.clear_displays()
        
        # Read all data types
        self.operation_requested.emit('read_encoder', {'motor_id': motor_id})
        self.operation_requested.emit('read_speed', {'motor_id': motor_id})
        self.operation_requested.emit('read_io_status', {'motor_id': motor_id})
        self.operation_requested.emit('read_motor_status', {'motor_id': motor_id})
    
    def update_encoder_display(self, data):
        """Update encoder display with new data.
        
        Args:
            data: Dictionary with encoder data
        """
        encoder_text = f"""<b style="color: #1976D2;">Encoder Data:</b><br>
<b>Raw Value:</b> {data.get('encoder_raw_value', 0)}<br>
<b>Angle (Degrees):</b> {data.get('encoder_angle_degrees', 0):.2f}°<br>
<b>Angle (Radians):</b> {data.get('encoder_angle_radians', 0):.4f} rad"""
        self.encoder_display.setHtml(encoder_text)
    
    def update_speed_display(self, data):
        """Update speed display with new data.
        
        Args:
            data: Dictionary with speed data
        """
        speed_text = f"""<b style="color: #1976D2;">Speed Data:</b><br>
<b>Speed (RPM):</b> {data.get('speed_rpm', 0)}<br>
<b>Speed (rad/s):</b> {data.get('speed_rad_per_sec', 0):.4f} rad/s"""
        self.speed_display.setHtml(speed_text)
    
    def update_io_display(self, data):
        """Update I/O display with new data.
        
        Args:
            data: Dictionary with I/O data
        """
        io_text = f"""<b style="color: #1976D2;">IO Status:</b><br>
<b>IN1 (Home/Left Limit):</b> <span style="color: {'green' if data.get('io_in1') else 'red'};">{'HIGH' if data.get('io_in1') else 'LOW'}</span><br>
<b>IN2 (Right Limit):</b> <span style="color: {'green' if data.get('io_in2') else 'red'};">{'HIGH' if data.get('io_in2') else 'LOW'}</span><br>
<b>OUT1 (Stall Detection):</b> <span style="color: {'green' if data.get('io_out1') else 'red'};">{'HIGH' if data.get('io_out1') else 'LOW'}</span><br>
<b>OUT2:</b> <span style="color: {'green' if data.get('io_out2') else 'red'};">{'HIGH' if data.get('io_out2') else 'LOW'}</span><br>
<b>Stall Detected:</b> <span style="color: {'red' if data.get('stall_detected') else 'green'};">{'YES' if data.get('stall_detected') else 'NO'}</span>"""
        self.io_display.setHtml(io_text)
    
    def update_motor_status_display(self, data):
        """Update motor status display with new data.
        
        Args:
            data: Dictionary with motor status data
        """
        status_text = f"""<b style="color: #1976D2;">Motor Status:</b><br>
<b>Enabled:</b> <span style="color: {'green' if data.get('motor_enabled') else 'red'};">{'YES' if data.get('motor_enabled') else 'NO'}</span><br>
<b>Moving:</b> <span style="color: {'orange' if data.get('motor_moving') else 'green'};">{'YES' if data.get('motor_moving') else 'NO'}</span><br>
<b>Calibrated:</b> <span style="color: {'green' if data.get('motor_calibrated') else 'red'};">{'YES' if data.get('motor_calibrated') else 'NO'}</span><br>
<b>Error:</b> <span style="color: {'red' if data.get('motor_error') else 'green'};">{'YES' if data.get('motor_error') else 'NO'}</span><br>
<b>Status Code:</b> {data.get('motor_status_code', 0)}<br>
<b>Status:</b> {data.get('motor_status_text', 'Unknown')}"""
        self.motor_status_display.setHtml(status_text)
