"""Motor control tab for MKS servo motors."""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, 
    QPushButton, QComboBox, QFormLayout, QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal


class MotorControlTab(QWidget):
    """Motor control tab for MKS motors.
    
    Provides controls for motor enable/disable and status query.
    """
    
    operation_requested = pyqtSignal(str, dict)  # operation_name, parameters
    
    def __init__(self, parent=None):
        """Initialize the motor control tab.
        
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
        
        # Motor Control Group
        control_group = QGroupBox("Motor Control")
        control_layout = QVBoxLayout()
        control_layout.setSpacing(15)
        
        # Enable/Disable buttons
        enable_layout = QHBoxLayout()
        enable_layout.setSpacing(10)
        
        self.enable_btn = QPushButton("Enable Motor")
        self.enable_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 15px;
                border-radius: 4px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.enable_btn.clicked.connect(lambda: self._on_enable_motor(True))
        enable_layout.addWidget(self.enable_btn)
        
        self.disable_btn = QPushButton("Disable Motor")
        self.disable_btn.setStyleSheet("""
            QPushButton {
                background-color: #F44336;
                color: white;
                font-weight: bold;
                padding: 15px;
                border-radius: 4px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.disable_btn.clicked.connect(lambda: self._on_enable_motor(False))
        enable_layout.addWidget(self.disable_btn)
        
        control_layout.addLayout(enable_layout)
        
        # Status query button
        status_btn = QPushButton("Query Motor Status")
        status_btn.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                font-weight: bold;
                padding: 12px;
                border-radius: 4px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
        """)
        status_btn.clicked.connect(self._on_query_status)
        control_layout.addWidget(status_btn)
        
        # Info label
        control_info = QLabel(
            "• Enable/Disable: Turn motor power on or off\n"
            "• Query Status: Read current motor status information"
        )
        control_info.setStyleSheet("color: #666; font-size: 12px; margin-top: 10px;")
        control_layout.addWidget(control_info)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
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
    
    def _on_enable_motor(self, enable):
        """Handle motor enable/disable button click.
        
        Args:
            enable: True to enable, False to disable motor
        """
        motor_id = self.get_motor_id()
        action = "Enabling" if enable else "Disabling"
        
        # Confirmation for disable
        if not enable:
            reply = QMessageBox.question(
                self, 'Confirm Disable',
                f'Disable motor {motor_id}?\nThe motor will lose power.',
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return
        
        self.operation_requested.emit('enable_motor', {'motor_id': motor_id, 'enable': enable})
    
    def _on_query_status(self):
        """Handle motor status query button click."""
        motor_id = self.get_motor_id()
        self.operation_requested.emit('query_status', {'motor_id': motor_id})
