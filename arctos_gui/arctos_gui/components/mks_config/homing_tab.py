"""Homing configuration tab for MKS servo motors."""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, 
    QPushButton, QSpinBox, QComboBox, QCheckBox, QFormLayout, QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal


class HomingTab(QWidget):
    """Homing tab for MKS motor configuration.
    
    Provides controls for homing parameters and homing operations.
    """
    
    operation_requested = pyqtSignal(str, dict)  # operation_name, parameters
    
    def __init__(self, parent=None):
        """Initialize the homing tab.
        
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
        
        # Homing Parameters Group
        home_params_group = QGroupBox("Homing Parameters")
        home_params_layout = QFormLayout()
        home_params_layout.setSpacing(10)
        home_params_layout.setLabelAlignment(Qt.AlignRight)
        
        self.home_trigger_combo = QComboBox()
        self.home_trigger_combo.addItems(["0 - Low Level", "1 - High Level"])
        home_params_layout.addRow("Trigger Level:", self.home_trigger_combo)
        
        self.home_direction_combo = QComboBox()
        self.home_direction_combo.addItems(["0 - Clockwise", "1 - Counter-Clockwise"])
        home_params_layout.addRow("Direction:", self.home_direction_combo)
        
        self.home_speed_spin = QSpinBox()
        self.home_speed_spin.setRange(1, 1000)
        self.home_speed_spin.setValue(100)
        self.home_speed_spin.setSuffix(" RPM")
        home_params_layout.addRow("Homing Speed:", self.home_speed_spin)
        
        self.home_enable_limit_check = QCheckBox("Enable Limit Switch")
        self.home_enable_limit_check.setChecked(True)
        home_params_layout.addRow("", self.home_enable_limit_check)
        
        set_params_btn = QPushButton("Set Homing Parameters")
        set_params_btn.clicked.connect(self._on_set_home_params)
        home_params_layout.addRow("", set_params_btn)
        home_params_group.setLayout(home_params_layout)
        layout.addWidget(home_params_group)
        
        # Homing Operations Group
        home_ops_group = QGroupBox("Homing Operations")
        home_ops_layout = QVBoxLayout()
        home_ops_layout.setSpacing(15)
        
        # Homing buttons in horizontal layout
        home_buttons_layout = QHBoxLayout()
        
        self.home_btn = QPushButton("Start Homing")
        self.home_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                padding: 12px;
                border-radius: 4px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        self.home_btn.clicked.connect(self._on_go_home)
        home_buttons_layout.addWidget(self.home_btn)
        
        self.set_zero_btn = QPushButton("Set Current Position as Zero")
        self.set_zero_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                font-weight: bold;
                padding: 12px;
                border-radius: 4px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
        """)
        self.set_zero_btn.clicked.connect(self._on_set_zero)
        home_buttons_layout.addWidget(self.set_zero_btn)
        
        home_ops_layout.addLayout(home_buttons_layout)
        
        # Info label
        home_info = QLabel(
            "• Start Homing: Moves the motor to the home/limit switch\n"
            "• Set Zero: Marks the current position as zero reference"
        )
        home_info.setStyleSheet("color: #666; font-size: 12px; margin-top: 10px;")
        home_ops_layout.addWidget(home_info)
        
        home_ops_group.setLayout(home_ops_layout)
        layout.addWidget(home_ops_group)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _on_set_home_params(self):
        """Handle homing parameters setting button click."""
        trigger = self.home_trigger_combo.currentIndex()
        direction = self.home_direction_combo.currentIndex()
        speed = self.home_speed_spin.value()
        enable_limit = self.home_enable_limit_check.isChecked()
        
        params = {
            'trigger': trigger,
            'direction': direction,
            'speed': speed,
            'enable_limit': enable_limit
        }
        self.operation_requested.emit('set_home_params', params)
    
    def _on_go_home(self):
        """Handle homing sequence start button click."""
        reply = QMessageBox.question(
            self, 'Confirm Homing',
            'Start the homing sequence?\nThe motor will move to find the home position.',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.operation_requested.emit('go_home', {})
    
    def _on_set_zero(self):
        """Handle zero position setting button click."""
        reply = QMessageBox.question(
            self, 'Confirm Zero Position',
            'Set the current position as zero reference?',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.operation_requested.emit('set_zero', {})
