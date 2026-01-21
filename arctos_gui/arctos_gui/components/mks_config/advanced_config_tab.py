"""Advanced configuration tab for MKS servo motors."""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QLabel, QPushButton,
    QCheckBox, QFormLayout, QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal


class AdvancedConfigTab(QWidget):
    """Advanced tab for MKS motor configuration.
    
    Provides controls for limit switch remapping and factory reset.
    """
    
    operation_requested = pyqtSignal(str, dict)  # operation_name, parameters
    
    def __init__(self, parent=None):
        """Initialize the advanced configuration tab.
        
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
        
        # Limit Switch Group
        limit_group = QGroupBox("Limit Switch Configuration")
        limit_layout = QVBoxLayout()
        limit_layout.setSpacing(10)
        
        limit_info = QLabel(
            "Enable limit switch remapping to use external limit switches "
            "instead of built-in homing switches."
        )
        limit_info.setWordWrap(True)
        limit_info.setStyleSheet("color: #666; font-size: 12px; margin-bottom: 10px;")
        limit_layout.addWidget(limit_info)
        
        self.limit_remap_check = QCheckBox("Enable Limit Switch Remapping")
        self.limit_remap_check.setChecked(False)
        self.limit_remap_check.stateChanged.connect(self._on_limit_remap_changed)
        limit_layout.addWidget(self.limit_remap_check)
        
        limit_group.setLayout(limit_layout)
        layout.addWidget(limit_group)
        
        # Factory Reset Group
        restore_group = QGroupBox("Factory Reset")
        restore_layout = QVBoxLayout()
        restore_layout.setSpacing(10)
        
        restore_warning = QLabel("⚠️ This will restore all motor parameters to factory defaults!")
        restore_warning.setStyleSheet("color: #FF5722; font-weight: bold;")
        restore_layout.addWidget(restore_warning)
        
        restore_info = QLabel(
            "All custom settings including work mode, currents, subdivisions, "
            "and homing parameters will be lost."
        )
        restore_info.setWordWrap(True)
        restore_info.setStyleSheet("color: #666; font-size: 12px; margin: 10px 0;")
        restore_layout.addWidget(restore_info)
        
        restore_btn = QPushButton("Restore Factory Defaults")
        restore_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF5722;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #E64A19;
            }
        """)
        restore_btn.clicked.connect(self._on_restore_defaults)
        restore_layout.addWidget(restore_btn)
        
        restore_group.setLayout(restore_layout)
        layout.addWidget(restore_group)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _on_limit_remap_changed(self, state):
        """Handle limit remap checkbox change."""
        enable = state == Qt.Checked
        self.operation_requested.emit('set_limit_remap', {'enable': enable})
    
    def _on_restore_defaults(self):
        """Handle factory reset button click with confirmation dialog."""
        reply = QMessageBox.warning(
            self, 'Confirm Factory Reset',
            'Are you sure you want to restore factory defaults?\n\n'
            'All custom settings will be lost!',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.operation_requested.emit('restore_defaults', {})
