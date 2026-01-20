from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, 
    QPushButton, QSpinBox, QComboBox, QCheckBox, QTabWidget,
    QFormLayout, QMessageBox, QProgressBar, QTextEdit, QScrollArea,
    QGridLayout, QFrame
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor
from .mks_config_client import MKSConfigClient


class MKSConfigWidget(QWidget):
    
    status_updated = pyqtSignal(str, bool)
    
    def __init__(self, mks_client: MKSConfigClient, parent=None):
        super().__init__(parent)
        self.mks_client = mks_client
        self.pending_futures = []
        self._connected = False
        
        # Motor mapping: Joint name -> Motor ID
        self.motor_mapping = {
            'X - Joint 1': 1,
            'Y - Joint 2': 2,
            'Z - Joint 3': 3,
            'A - Joint 4': 4,
            'B - Joint 5': 5,
            'C - Joint 6': 6
        }
        
        self.init_ui()
        self.apply_modern_style()
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.check_pending_responses)
        self.update_timer.start(100)
        
        # Add connection status timer
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self.update_connection_status)
        self.connection_timer.start(1000)  # Check every second
    
    def init_ui(self):
        # Main layout with scroll area for scalability
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Title
        title_label = QLabel("MKS Servo Motor Configuration")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #1976D2; padding: 5px;")
        main_layout.addWidget(title_label)
        
        # Connection Status Group
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
        self._interface_label = QLabel("Interface: can0")
        self._interface_label.setStyleSheet("""
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
        info_row.addWidget(self._interface_label)
        info_row.addStretch()
        info_row.addWidget(self._last_update)
        status_layout.addLayout(info_row)
        
        main_layout.addWidget(status_group)
        
        # Motor selection - compact layout
        motor_selection_group = QGroupBox("Motor Selection")
        motor_selection_layout = QHBoxLayout()
        motor_selection_layout.setSpacing(10)
        
        motor_label = QLabel("Motor:")
        motor_label_font = QFont()
        motor_label_font.setPointSize(10)
        motor_label_font.setBold(True)
        motor_label.setFont(motor_label_font)
        
        self.motor_combo = QComboBox()
        self.motor_combo.addItems(list(self.motor_mapping.keys()))
        self.motor_combo.setMinimumWidth(150)
        self.motor_combo.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        
        motor_id_display = QLabel("ID:")
        self.motor_id_value = QLabel("1")
        self.motor_id_value.setStyleSheet("font-weight: bold; color: #1976D2; font-size: 10pt;")
        
        self.motor_combo.currentTextChanged.connect(self.on_motor_selection_changed)
        
        motor_selection_layout.addWidget(motor_label)
        motor_selection_layout.addWidget(self.motor_combo)
        motor_selection_layout.addSpacing(15)
        motor_selection_layout.addWidget(motor_id_display)
        motor_selection_layout.addWidget(self.motor_id_value)
        motor_selection_layout.addStretch()
        
        motor_selection_group.setLayout(motor_selection_layout)
        main_layout.addWidget(motor_selection_group)
        
        # Tabs with scroll areas for each tab content
        self.tab_widget = QTabWidget()
        self.tab_widget.addTab(self.create_basic_config_tab(), "Basic Config")
        self.tab_widget.addTab(self.create_advanced_config_tab(), "Advanced")
        self.tab_widget.addTab(self.create_homing_tab(), "Homing")
        self.tab_widget.addTab(self.create_motor_control_tab(), "Control")
        self.tab_widget.addTab(self.create_read_data_tab(), "Read Data")
        main_layout.addWidget(self.tab_widget, 1)
        
        # Status log - compact
        status_label = QLabel("Status Log")
        status_label_font = QFont()
        status_label_font.setPointSize(10)
        status_label_font.setBold(True)
        status_label.setFont(status_label_font)
        status_label.setStyleSheet("color: #424242;")
        main_layout.addWidget(status_label)
        
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(100)
        self.status_text.setPlaceholderText("Status messages will appear here...")
        main_layout.addWidget(self.status_text)
        
        self.setLayout(main_layout)
    
    def create_basic_config_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        work_mode_group = QGroupBox("Work Mode")
        work_mode_layout = QFormLayout()
        work_mode_layout.setSpacing(10)
        work_mode_layout.setLabelAlignment(Qt.AlignRight)
        self.work_mode_combo = QComboBox()
        self.work_mode_combo.addItems([
            "0 - CR (Closed Loop, Current Control)",
            "1 - CR_vFOC (Closed Loop, Current Control, FOC)",
            "2 - SR_OPEN (Open Loop, Speed Control)",
            "3 - SR_CLOSE (Closed Loop, Speed Control)",
            "4 - SR_FOC (Closed Loop, Speed Control, FOC)",
            "5 - SR_vFOC (Closed Loop, Speed Control, FOC + Encoder)"
        ])
        self.work_mode_combo.setCurrentIndex(5)
        work_mode_layout.addRow("Mode:", self.work_mode_combo)
        
        set_mode_btn = QPushButton("Set Work Mode")
        set_mode_btn.clicked.connect(self.on_set_work_mode)
        work_mode_layout.addRow("", set_mode_btn)
        work_mode_group.setLayout(work_mode_layout)
        layout.addWidget(work_mode_group)
        
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
        set_current_btn.clicked.connect(self.on_set_working_current)
        current_layout.addRow("", set_current_btn)
        
        self.holding_current_spin = QSpinBox()
        self.holding_current_spin.setRange(0, 100)
        self.holding_current_spin.setValue(70)
        self.holding_current_spin.setSuffix(" %")
        current_layout.addRow("Holding Current:", self.holding_current_spin)
        
        set_holding_btn = QPushButton("Set Holding Current")
        set_holding_btn.clicked.connect(self.on_set_holding_current)
        current_layout.addRow("", set_holding_btn)
        
        current_group.setLayout(current_layout)
        layout.addWidget(current_group)
        
        calibrate_group = QGroupBox("Motor Calibration")
        calibrate_layout = QVBoxLayout()
        calibrate_layout.setSpacing(10)
        calibrate_info = QLabel("⚠️ Calibration will rotate the motor to establish encoder reference.")
        calibrate_info.setWordWrap(True)
        calibrate_info.setStyleSheet("color: #FF9800; padding: 5px;")
        calibrate_layout.addWidget(calibrate_info)
        
        calibrate_btn = QPushButton("Calibrate Motor")
        calibrate_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        calibrate_btn.clicked.connect(self.on_calibrate_motor)
        calibrate_layout.addWidget(calibrate_btn)
        
        calibrate_group.setLayout(calibrate_layout)
        layout.addWidget(calibrate_group)
        
        layout.addStretch()
        widget.setLayout(layout)
        scroll.setWidget(widget)
        return scroll
    
    def create_advanced_config_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
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
        set_subdivision_btn.clicked.connect(self.on_set_subdivision)
        subdivision_layout.addRow("", set_subdivision_btn)
        
        subdivision_group.setLayout(subdivision_layout)
        layout.addWidget(subdivision_group)
        
        limit_group = QGroupBox("Limit Switch Configuration")
        limit_layout = QFormLayout()
        self.limit_remap_check = QCheckBox("Enable Limit Port Remapping")
        limit_layout.addRow("", self.limit_remap_check)
        
        set_limit_btn = QPushButton("Set Limit Remap")
        set_limit_btn.clicked.connect(self.on_set_limit_remap)
        limit_layout.addRow("", set_limit_btn)
        
        limit_group.setLayout(limit_layout)
        layout.addWidget(limit_group)
        
        restore_group = QGroupBox("Factory Reset")
        restore_layout = QVBoxLayout()
        restore_warning = QLabel("⚠️ This will restore all motor parameters to factory defaults!")
        restore_warning.setStyleSheet("color: #FF5722; font-weight: bold;")
        restore_layout.addWidget(restore_warning)
        
        restore_btn = QPushButton("Restore Factory Defaults")
        restore_btn.setStyleSheet("background-color: #FF5722; color: white; font-weight: bold; padding: 10px;")
        restore_btn.clicked.connect(self.on_restore_defaults)
        restore_layout.addWidget(restore_btn)
        
        restore_group.setLayout(restore_layout)
        layout.addWidget(restore_group)
        
        layout.addStretch()
        widget.setLayout(layout)
        scroll.setWidget(widget)
        return scroll
    
    def create_homing_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
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
        self.home_speed_spin.setRange(1, 3000)
        self.home_speed_spin.setValue(300)
        self.home_speed_spin.setSuffix(" RPM")
        home_params_layout.addRow("Speed:", self.home_speed_spin)
        
        self.home_enable_limit_check = QCheckBox("Enable Limit Switch")
        self.home_enable_limit_check.setChecked(True)
        home_params_layout.addRow("", self.home_enable_limit_check)
        
        set_home_params_btn = QPushButton("Set Homing Parameters")
        set_home_params_btn.clicked.connect(self.on_set_home_params)
        home_params_layout.addRow("", set_home_params_btn)
        
        home_params_group.setLayout(home_params_layout)
        layout.addWidget(home_params_group)
        
        home_actions_group = QGroupBox("Homing Actions")
        home_actions_layout = QVBoxLayout()
        home_actions_layout.setSpacing(10)
        
        go_home_btn = QPushButton("Start Homing Sequence")
        go_home_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        go_home_btn.clicked.connect(self.on_go_home)
        home_actions_layout.addWidget(go_home_btn)
        
        set_zero_btn = QPushButton("Set Current Position as Zero")
        set_zero_btn.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 10px;")
        set_zero_btn.clicked.connect(self.on_set_zero)
        home_actions_layout.addWidget(set_zero_btn)
        
        home_actions_group.setLayout(home_actions_layout)
        layout.addWidget(home_actions_group)
        
        layout.addStretch()
        widget.setLayout(layout)
        scroll.setWidget(widget)
        return scroll
    
    def create_motor_control_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        enable_group = QGroupBox("Motor Enable/Disable")
        enable_layout = QHBoxLayout()
        enable_layout.setSpacing(15)
        
        enable_btn = QPushButton("Enable Motor")
        enable_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 15px;")
        enable_btn.clicked.connect(lambda: self.on_enable_motor(True))
        enable_layout.addWidget(enable_btn)
        
        disable_btn = QPushButton("Disable Motor")
        disable_btn.setStyleSheet("background-color: #F44336; color: white; font-weight: bold; padding: 15px;")
        disable_btn.clicked.connect(lambda: self.on_enable_motor(False))
        enable_layout.addWidget(disable_btn)
        
        enable_group.setLayout(enable_layout)
        layout.addWidget(enable_group)
        
        status_group = QGroupBox("Motor Status")
        status_layout = QVBoxLayout()
        status_layout.setSpacing(10)
        
        query_status_btn = QPushButton("Query Motor Status")
        query_status_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        query_status_btn.clicked.connect(self.on_query_status)
        status_layout.addWidget(query_status_btn)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
        widget.setLayout(layout)
        scroll.setWidget(widget)
        return scroll
    
    def create_read_data_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Encoder Data Section
        encoder_group = QGroupBox("Encoder Position")
        encoder_layout = QVBoxLayout()
        encoder_layout.setSpacing(10)
        
        encoder_btn_layout = QHBoxLayout()
        read_encoder_btn = QPushButton("Read Encoder")
        read_encoder_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        read_encoder_btn.clicked.connect(self.on_read_encoder)
        encoder_btn_layout.addWidget(read_encoder_btn)
        encoder_btn_layout.addStretch()
        encoder_layout.addLayout(encoder_btn_layout)
        
        self.encoder_display = QTextEdit()
        self.encoder_display.setReadOnly(True)
        self.encoder_display.setMaximumHeight(100)
        self.encoder_display.setPlaceholderText("Encoder data will appear here...")
        encoder_layout.addWidget(self.encoder_display)
        encoder_group.setLayout(encoder_layout)
        layout.addWidget(encoder_group)
        
        # Speed Data Section
        speed_group = QGroupBox("Motor Speed")
        speed_layout = QVBoxLayout()
        speed_layout.setSpacing(10)
        
        speed_btn_layout = QHBoxLayout()
        read_speed_btn = QPushButton("Read Speed")
        read_speed_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        read_speed_btn.clicked.connect(self.on_read_speed)
        speed_btn_layout.addWidget(read_speed_btn)
        speed_btn_layout.addStretch()
        speed_layout.addLayout(speed_btn_layout)
        
        self.speed_display = QTextEdit()
        self.speed_display.setReadOnly(True)
        self.speed_display.setMaximumHeight(80)
        self.speed_display.setPlaceholderText("Speed data will appear here...")
        speed_layout.addWidget(self.speed_display)
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)
        
        # IO Status Section
        io_group = QGroupBox("IO Status")
        io_layout = QVBoxLayout()
        io_layout.setSpacing(10)
        
        io_btn_layout = QHBoxLayout()
        read_io_btn = QPushButton("Read IO Status")
        read_io_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        read_io_btn.clicked.connect(self.on_read_io_status)
        io_btn_layout.addWidget(read_io_btn)
        io_btn_layout.addStretch()
        io_layout.addLayout(io_btn_layout)
        
        self.io_display = QTextEdit()
        self.io_display.setReadOnly(True)
        self.io_display.setMaximumHeight(100)
        self.io_display.setPlaceholderText("IO status will appear here...")
        io_layout.addWidget(self.io_display)
        io_group.setLayout(io_layout)
        layout.addWidget(io_group)
        
        # Motor Status Section
        status_group = QGroupBox("Motor Status")
        status_layout = QVBoxLayout()
        status_layout.setSpacing(10)
        
        status_btn_layout = QHBoxLayout()
        read_status_btn = QPushButton("Read Motor Status")
        read_status_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        read_status_btn.clicked.connect(self.on_read_motor_status)
        status_btn_layout.addWidget(read_status_btn)
        status_btn_layout.addStretch()
        status_layout.addLayout(status_btn_layout)
        
        self.motor_status_display = QTextEdit()
        self.motor_status_display.setReadOnly(True)
        self.motor_status_display.setMaximumHeight(100)
        self.motor_status_display.setPlaceholderText("Motor status will appear here...")
        status_layout.addWidget(self.motor_status_display)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
        widget.setLayout(layout)
        scroll.setWidget(widget)
        return scroll
    
    def apply_modern_style(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #f5f5f5;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 10pt;
            }
            QGroupBox {
                border: 1px solid #e0e0e0;
                border-radius: 5px;
                margin-top: 8px;
                padding-top: 12px;
                font-weight: bold;
                background-color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #1976D2;
            }
            QPushButton {
                background-color: #1976D2;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                min-height: 25px;
            }
            QPushButton:hover {
                background-color: #1565C0;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
            QPushButton:disabled {
                background-color: #BDBDBD;
            }
            QSpinBox, QComboBox {
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                padding: 4px;
                background-color: white;
                min-height: 22px;
                color: black;
            }
            QSpinBox:hover, QComboBox:hover {
                border: 1px solid #1976D2;
                background-color: white;
            }
            QSpinBox:focus, QComboBox:focus {
                border: 2px solid #1976D2;
                background-color: white;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 4px solid transparent;
                border-right: 4px solid transparent;
                border-top: 6px solid #424242;
                width: 0;
                height: 0;
            }
            QComboBox QAbstractItemView {
                border: 1px solid #e0e0e0;
                background-color: white;
                selection-background-color: #1976D2;
                selection-color: white;
                color: black;
            }
            QComboBox QAbstractItemView::item {
                padding: 4px;
                background-color: white;
                color: black;
            }
            QComboBox QAbstractItemView::item:hover {
                background-color: #E3F2FD;
                color: black;
            }
            QComboBox QAbstractItemView::item:selected {
                background-color: #1976D2;
                color: white;
            }
            QTextEdit {
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                background-color: white;
                padding: 4px;
            }
            QTabWidget::pane {
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                background-color: white;
            }
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QTabBar::tab {
                background-color: #e0e0e0;
                border: 1px solid #bdbdbd;
                border-bottom: none;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
                padding: 8px 15px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: white;
                border-bottom: 2px solid white;
            }
            QTabBar::tab:hover {
                background-color: #d0d0d0;
            }
            QLabel {
                color: #424242;
            }
        """)
    
    def log_status(self, message: str, success: bool = True):
        color = "green" if success else "red"
        self.status_text.append(f'<span style="color: {color};">[{self.get_timestamp()}] {message}</span>')
        self.status_updated.emit(message, success)
    
    def get_timestamp(self):
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S")
    
    def update_connection_status(self):
        """Check and update connection status by testing if services are responsive."""
        if not self.mks_client:
            return
            
        try:
            # Check if all services are ready - this indicates we're connected
            config_ready = self.mks_client.config_client.service_is_ready()
            read_ready = self.mks_client.read_client.service_is_ready()
            connection_ready = self.mks_client.connection_client.service_is_ready()
            
            # If all services are ready, we're connected
            all_ready = config_ready and read_ready and connection_ready
            
            if all_ready != self._connected:
                # Status changed, update display
                if all_ready:
                    self._update_connection_status(True, "All services ready")
                else:
                    self._update_connection_status(False, "Services not ready")
                    
        except Exception as e:
            # Any exception means we're not connected
            if self._connected:
                self._update_connection_status(False, f"Connection error: {str(e)}")
    
    def get_motor_id(self):
        selected_motor = self.motor_combo.currentText()
        return self.motor_mapping[selected_motor]
    
    def on_motor_selection_changed(self, motor_name):
        motor_id = self.motor_mapping[motor_name]
        self.motor_id_value.setText(str(motor_id))
    
    def on_set_work_mode(self):
        motor_id = self.get_motor_id()
        work_mode = self.work_mode_combo.currentIndex()
        self.log_status(f"Setting work mode {work_mode} for motor {motor_id}...")
        future = self.mks_client.set_work_mode(motor_id, work_mode)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_set_working_current(self):
        motor_id = self.get_motor_id()
        current = self.working_current_spin.value()
        self.log_status(f"Setting working current {current}mA for motor {motor_id}...")
        future = self.mks_client.set_working_current(motor_id, current)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_set_holding_current(self):
        motor_id = self.get_motor_id()
        percentage = self.holding_current_spin.value()
        self.log_status(f"Setting holding current {percentage}% for motor {motor_id}...")
        future = self.mks_client.set_holding_current(motor_id, percentage)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_calibrate_motor(self):
        motor_id = self.get_motor_id()
        reply = QMessageBox.question(
            self, 'Confirm Calibration',
            f'Start calibration for motor {motor_id}?\nThe motor will rotate during calibration.',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.log_status(f"Starting calibration for motor {motor_id}...")
            future = self.mks_client.calibrate_motor(motor_id)
            if future:
                self.pending_futures.append(('config', future))
    
    def on_set_subdivision(self):
        motor_id = self.get_motor_id()
        subdivision = self.subdivision_combo.currentIndex()
        self.log_status(f"Setting subdivision {subdivision} for motor {motor_id}...")
        future = self.mks_client.set_subdivision(motor_id, subdivision)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_set_limit_remap(self):
        motor_id = self.get_motor_id()
        enable = self.limit_remap_check.isChecked()
        self.log_status(f"Setting limit remap {'enabled' if enable else 'disabled'} for motor {motor_id}...")
        future = self.mks_client.set_limit_remap(motor_id, enable)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_restore_defaults(self):
        motor_id = self.get_motor_id()
        reply = QMessageBox.warning(
            self, 'Confirm Factory Reset',
            f'Restore factory defaults for motor {motor_id}?\nAll custom settings will be lost!',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.log_status(f"Restoring factory defaults for motor {motor_id}...")
            future = self.mks_client.restore_defaults(motor_id)
            if future:
                self.pending_futures.append(('config', future))
    
    def on_set_home_params(self):
        motor_id = self.get_motor_id()
        trigger = self.home_trigger_combo.currentIndex()
        direction = self.home_direction_combo.currentIndex()
        speed = self.home_speed_spin.value()
        enable_limit = self.home_enable_limit_check.isChecked()
        
        self.log_status(f"Setting homing parameters for motor {motor_id}...")
        future = self.mks_client.set_home_parameters(motor_id, trigger, direction, speed, enable_limit)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_go_home(self):
        motor_id = self.get_motor_id()
        self.log_status(f"Starting homing sequence for motor {motor_id}...")
        future = self.mks_client.go_home(motor_id)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_set_zero(self):
        motor_id = self.get_motor_id()
        self.log_status(f"Setting zero position for motor {motor_id}...")
        future = self.mks_client.set_zero_position(motor_id)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_enable_motor(self, enable: bool):
        motor_id = self.get_motor_id()
        action = "Enabling" if enable else "Disabling"
        self.log_status(f"{action} motor {motor_id}...")
        future = self.mks_client.enable_motor(motor_id, enable)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_query_status(self):
        motor_id = self.get_motor_id()
        self.log_status(f"Querying status for motor {motor_id}...")
        future = self.mks_client.query_status(motor_id)
        if future:
            self.pending_futures.append(('config', future))
    
    def on_read_encoder(self):
        motor_id = self.get_motor_id()
        self.log_status(f"Reading encoder for motor {motor_id}...")
        future = self.mks_client.read_encoder(motor_id)
        if future:
            self.pending_futures.append(('read', future))
    
    def on_read_speed(self):
        motor_id = self.get_motor_id()
        self.log_status(f"Reading speed for motor {motor_id}...")
        future = self.mks_client.read_speed(motor_id)
        if future:
            self.pending_futures.append(('read', future))
    
    def on_read_io_status(self):
        motor_id = self.get_motor_id()
        self.log_status(f"Reading IO status for motor {motor_id}...")
        future = self.mks_client.read_io_status(motor_id)
        if future:
            self.pending_futures.append(('read', future))
    
    def on_read_motor_status(self):
        motor_id = self.get_motor_id()
        self.log_status(f"Reading motor status for motor {motor_id}...")
        future = self.mks_client.read_motor_status(motor_id)
        if future:
            self.pending_futures.append(('read', future))
    
    def check_pending_responses(self):
        completed = []
        for i, (response_type, future) in enumerate(self.pending_futures):
            if future.done():
                try:
                    response = future.result()
                    if response_type == 'config':
                        self.handle_config_response(response)
                    elif response_type == 'read':
                        self.handle_read_response(response)
                    elif response_type == 'connection':
                        self.handle_connection_response(response)
                except Exception as e:
                    self.log_status(f"Error: {str(e)}", False)
                completed.append(i)
        
        for i in reversed(completed):
            self.pending_futures.pop(i)
    
    def handle_config_response(self, response):
        if response.success:
            self.log_status(f"✓ {response.message}", True)
        else:
            self.log_status(f"✗ {response.message}", False)
    
    def handle_connection_response(self, response):
        self._update_connection_status(response.success, response.message)
    
    def handle_read_response(self, response):
        if response.success:
            self.log_status(f"✓ {response.message}", True)
            
            # Display encoder data - always show when we have encoder fields
            if hasattr(response, 'encoder_raw_value'):
                # Always display if we have encoder data fields, regardless of values
                if 'encoder' in response.message.lower():
                    encoder_text = f"""<b style="color: #1976D2;">Encoder Data:</b><br>
<b>Raw Value:</b> {response.encoder_raw_value}<br>
<b>Angle (Degrees):</b> {response.encoder_angle_degrees:.2f}°<br>
<b>Angle (Radians):</b> {response.encoder_angle_radians:.4f} rad"""
                    self.encoder_display.setHtml(encoder_text)
                    self.log_status(f"Encoder: Raw={response.encoder_raw_value}, Deg={response.encoder_angle_degrees:.2f}°, Rad={response.encoder_angle_radians:.4f}", True)
            
            # Display speed data
            if hasattr(response, 'speed_rpm'):
                # Always display if we have speed data fields, regardless of values
                if 'speed' in response.message.lower():
                    speed_text = f"""<b style="color: #1976D2;">Speed Data:</b><br>
<b>Speed (RPM):</b> {response.speed_rpm}<br>
<b>Speed (rad/s):</b> {response.speed_rad_per_sec:.4f} rad/s"""
                    self.speed_display.setHtml(speed_text)
            
            # Display IO status
            if hasattr(response, 'io_in1') and 'io' in response.message.lower():
                io_text = f"""<b style="color: #1976D2;">IO Status:</b><br>
<b>IN1 (Home/Left Limit):</b> <span style="color: {'green' if response.io_in1 else 'red'};">{'HIGH' if response.io_in1 else 'LOW'}</span><br>
<b>IN2 (Right Limit):</b> <span style="color: {'green' if response.io_in2 else 'red'};">{'HIGH' if response.io_in2 else 'LOW'}</span><br>
<b>OUT1 (Stall Detection):</b> <span style="color: {'green' if response.io_out1 else 'red'};">{'HIGH' if response.io_out1 else 'LOW'}</span><br>
<b>OUT2:</b> <span style="color: {'green' if response.io_out2 else 'red'};">{'HIGH' if response.io_out2 else 'LOW'}</span><br>
<b>Stall Detected:</b> <span style="color: {'red' if response.stall_detected else 'green'};">{'YES' if response.stall_detected else 'NO'}</span>"""
                self.io_display.setHtml(io_text)
            
            # Display motor status
            if hasattr(response, 'motor_enabled') and 'status' in response.message.lower():
                status_text = f"""<b style="color: #1976D2;">Motor Status:</b><br>
<b>Enabled:</b> <span style="color: {'green' if response.motor_enabled else 'red'};">{'YES' if response.motor_enabled else 'NO'}</span><br>
<b>Moving:</b> <span style="color: {'orange' if response.motor_moving else 'green'};">{'YES' if response.motor_moving else 'NO'}</span><br>
<b>Calibrated:</b> <span style="color: {'green' if response.motor_calibrated else 'red'};">{'YES' if response.motor_calibrated else 'NO'}</span><br>
<b>Error:</b> <span style="color: {'red' if response.motor_error else 'green'};">{'YES' if response.motor_error else 'NO'}</span><br>
<b>Status Code:</b> {response.motor_status_code}"""
                self.motor_status_display.setHtml(status_text)
        else:
            self.log_status(f"✗ {response.message}", False)
    
    def _toggle_connection(self):
        """Toggle CAN connection."""
        if not self.mks_client:
            return
            
        if self._connected:
            self.log_status("Disconnecting from CAN interface...")
            future = self.mks_client.disconnect_can()
            if future:
                self.pending_futures.append(('connection', future))
        else:
            self.log_status("Connecting to CAN interface...")
            future = self.mks_client.connect_can()
            if future:
                self.pending_futures.append(('connection', future))
    
    def _update_connection_status(self, connected: bool, message: str = ""):
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
                QPushButton:pressed {
                    background-color: #841f06;
                }
            """)
            self._last_update.setText(f"Last update: {self.get_timestamp()}")
            self.log_status(f"✓ Connected to CAN interface{': ' + message if message else ''}")
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
            if message:
                self.log_status(f"✗ Disconnected: {message}", False)
            else:
                self.log_status("✗ Disconnected from CAN interface", False)
