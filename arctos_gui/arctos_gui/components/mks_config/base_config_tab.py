"""Base configuration tab for MKS servo motors."""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, 
    QPushButton, QSpinBox, QComboBox, QFormLayout, QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont


class BaseConfigTab(QWidget):
    """Base tab for basic MKS motor configuration.
    
    Provides controls for work mode, currents, and subdivisions.
    """
    
    # Signal emitted when an operation is requested
    operation_requested = pyqtSignal(str, dict)  # operation_name, parameters
    
    def __init__(self, parent=None):
        """Initialize the base configuration tab.
        
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
        
        # Work Mode Group
        work_mode_group = QGroupBox("Work Mode")
        work_mode_layout = QFormLayout()
        work_mode_layout.setSpacing(10)
        work_mode_layout.setLabelAlignment(Qt.AlignRight)
        
        self.work_mode_combo = QComboBox()
        self.work_mode_combo.addItems([
            "0 - CR Open (Open Loop)",
            "1 - CR Close (Closed Loop)",
            "2 - CR_vFOC (Closed Loop FOC)",
            "3 - SR Open (Open Loop Speed)",
            "4 - SR Close (Closed Loop Speed)",
            "5 - SR_vFOC (Closed Loop Speed FOC)"
        ])
        self.work_mode_combo.setCurrentIndex(5)
        work_mode_layout.addRow("Mode:", self.work_mode_combo)
        
        set_mode_btn = QPushButton("Set Work Mode")
        set_mode_btn.clicked.connect(self._on_set_work_mode)
        work_mode_layout.addRow("", set_mode_btn)
        work_mode_group.setLayout(work_mode_layout)
        layout.addWidget(work_mode_group)
        
        # Current Settings Group
        current_group = QGroupBox("Current Settings")
        current_layout = QFormLayout()
        current_layout.setSpacing(10)
        current_layout.setLabelAlignment(Qt.AlignRight)
        
        self.working_current_spin = QSpinBox()
        self.working_current_spin.setRange(100, 3000)
        self.working_current_spin.setValue(1600)
        self.working_current_spin.setSuffix(" mA")
        current_layout.addRow("Working Current:", self.working_current_spin)
        
        set_current_btn = QPushButton("Set Working Current")
        set_current_btn.clicked.connect(self._on_set_working_current)
        current_layout.addRow("", set_current_btn)
        
        self.holding_current_spin = QSpinBox()
        self.holding_current_spin.setRange(10, 90)
        self.holding_current_spin.setSingleStep(10)
        self.holding_current_spin.setValue(70)
        self.holding_current_spin.setSuffix(" %")
        current_layout.addRow("Holding Current:", self.holding_current_spin)
        
        set_holding_btn = QPushButton("Set Holding Current")
        set_holding_btn.clicked.connect(self._on_set_holding_current)
        current_layout.addRow("", set_holding_btn)
        
        current_group.setLayout(current_layout)
        layout.addWidget(current_group)
        
        # Subdivision Group
        subdivision_group = QGroupBox("Microstep Subdivision")
        subdivision_layout = QFormLayout()
        subdivision_layout.setSpacing(10)
        subdivision_layout.setLabelAlignment(Qt.AlignRight)
        
        self.subdivision_combo = QComboBox()
        self.subdivision_combo.addItems([
            "0 - Full Step",
            "1 - 2 Microsteps",
            "2 - 4 Microsteps",
            "3 - 8 Microsteps",
            "4 - 16 Microsteps",
            "5 - 32 Microsteps",
            "6 - 64 Microsteps",
            "7 - 128 Microsteps",
            "8 - 256 Microsteps"
        ])
        self.subdivision_combo.setCurrentIndex(7)
        subdivision_layout.addRow("Subdivision:", self.subdivision_combo)
        
        set_subdivision_btn = QPushButton("Set Subdivision")
        set_subdivision_btn.clicked.connect(self._on_set_subdivision)
        subdivision_layout.addRow("", set_subdivision_btn)
        subdivision_group.setLayout(subdivision_layout)
        layout.addWidget(subdivision_group)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _on_set_work_mode(self):
        """Handle work mode setting button click."""
        work_mode = self.work_mode_combo.currentIndex()
        self.operation_requested.emit('set_work_mode', {'work_mode': work_mode})
    
    def _on_set_working_current(self):
        """Handle working current setting button click."""
        current = self.working_current_spin.value()
        self.operation_requested.emit('set_working_current', {'current_ma': current})
    
    def _on_set_holding_current(self):
        """Handle holding current setting button click."""
        percentage = self.holding_current_spin.value()
        self.operation_requested.emit('set_holding_current', {'percentage': percentage})
    
    def _on_set_subdivision(self):
        """Handle subdivision setting button click."""
        subdivision = self.subdivision_combo.currentIndex()
        self.operation_requested.emit('set_subdivision', {'subdivision': subdivision})
