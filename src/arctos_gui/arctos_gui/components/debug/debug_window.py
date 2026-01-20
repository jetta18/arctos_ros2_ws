"""Debug window widget for displaying STM32 debug messages."""

from datetime import datetime
from typing import Optional
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class DebugWindow(QWidget):
    """Widget for displaying real-time STM32 debug messages."""
    
    # Signal to handle text updates from different threads
    message_received = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        """Initialize debug window."""
        super().__init__(parent)
        self._max_lines = 1000
        self._is_paused = False
        self._message_count = 0
        self._client = None
        self._connected = False
        
        # Connect signal to slot
        self.message_received.connect(self._append_message)
        
        self._setup_ui()
        self._setup_timer()

    def _setup_ui(self) -> None:
        """Setup the user interface."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # Header with controls
        header = self._create_header()
        layout.addWidget(header)
        
        # Text display area
        self._text_display = QTextEdit()
        self._text_display.setReadOnly(True)
        self._text_display.setFont(self._get_monospace_font())
        self._text_display.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #444444;
                border-radius: 4px;
                padding: 5px;
            }
        """)
        layout.addWidget(self._text_display)
        
        # Status bar
        self._status_label = QLabel("Ready")
        self._status_label.setStyleSheet("""
            QLabel {
                color: #888888;
                font-size: 11px;
                padding: 2px;
            }
        """)
        layout.addWidget(self._status_label)

    def _create_header(self) -> QFrame:
        """Create header with control buttons."""
        header = QFrame()
        header.setFrameStyle(QFrame.StyledPanel)
        header.setStyleSheet("""
            QFrame {
                background-color: #2d2d2d;
                border-radius: 4px;
                padding: 5px;
            }
        """)
        
        layout = QHBoxLayout(header)
        layout.setContentsMargins(10, 5, 10, 5)
        
        # Title
        title = QLabel("STM32 Debug Output")
        title.setStyleSheet("""
            QLabel {
                color: #ffffff;
                font-weight: bold;
                font-size: 13px;
            }
        """)
        layout.addWidget(title)
        
        layout.addStretch()
        
        # Connection status and button
        self._connect_button = QPushButton("Connect")
        self._connect_button.clicked.connect(self._toggle_connection)
        self._connect_button.setStyleSheet("""
            QPushButton {
                background-color: #0078d4;
                color: white;
                border: none;
                padding: 5px 15px;
                border-radius: 3px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #106ebe;
            }
            QPushButton:pressed {
                background-color: #0e5a9a;
            }
        """)
        layout.addWidget(self._connect_button)
        
        # Control buttons
        self._pause_button = QPushButton("Pause")
        self._pause_button.setCheckable(True)
        self._pause_button.clicked.connect(self._toggle_pause)
        self._pause_button.setStyleSheet("""
            QPushButton {
                background-color: #444444;
                color: white;
                border: none;
                padding: 5px 15px;
                border-radius: 3px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #555555;
            }
            QPushButton:checked {
                background-color: #ff6b35;
            }
        """)
        layout.addWidget(self._pause_button)
        
        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self._clear_display)
        clear_button.setStyleSheet("""
            QPushButton {
                background-color: #444444;
                color: white;
                border: none;
                padding: 5px 15px;
                border-radius: 3px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #555555;
            }
        """)
        layout.addWidget(clear_button)
        
        return header

    def _setup_timer(self) -> None:
        """Setup status update timer."""
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(1000)  # Update every second

    def _get_monospace_font(self) -> QFont:
        """Get a monospace font for the text display."""
        font = QFont("Consolas", 9)
        if not font.exactMatch():
            font = QFont("Courier New", 9)
        return font

    def add_message(self, message: str) -> None:
        """Add a new debug message (thread-safe)."""
        if not self._is_paused:
            self.message_received.emit(message)
            self._message_count += 1

    def _append_message(self, message: str) -> None:
        """Append message to text display."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted_message = f"[{timestamp}] {message}"
        
        # Add color coding based on content
        if any(word in message.lower() for word in ['error', 'failed', 'fault']):
            formatted_message = f'<span style="color: #ff6b6b;">{formatted_message}</span>'
        elif any(word in message.lower() for word in ['warning', 'warn']):
            formatted_message = f'<span style="color: #feca57;">{formatted_message}</span>'
        elif any(word in message.lower() for word in ['info', 'status']):
            formatted_message = f'<span style="color: #48dbfb;">{formatted_message}</span>'
        else:
            formatted_message = f'<span style="color: #ffffff;">{formatted_message}</span>'
        
        self._text_display.append(formatted_message)
        
        # Limit number of lines
        if self._text_display.document().blockCount() > self._max_lines:
            cursor = self._text_display.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()  # Remove newline

    def _toggle_pause(self, checked: bool) -> None:
        """Toggle pause state."""
        self._is_paused = checked
        self._pause_button.setText("Resume" if checked else "Pause")

    def _clear_display(self) -> None:
        """Clear all messages."""
        self._text_display.clear()
        self._message_count = 0

    def _update_status(self) -> None:
        """Update status bar."""
        if self._is_paused:
            status = "Paused"
        elif self._connected:
            status = f"Connected | Messages: {self._message_count} | Lines: {self._text_display.document().blockCount()}"
        else:
            status = f"Disconnected | Messages: {self._message_count} | Lines: {self._text_display.document().blockCount()}"
        self._status_label.setText(status)
        
    def set_client(self, client) -> None:
        """Set the Ethernet debug client."""
        self._client = client
        
    def _toggle_connection(self) -> None:
        """Toggle Ethernet connection."""
        if not self._client:
            return
            
        if self._connected:
            self._client.disconnect()
            self._connected = False
            self._connect_button.setText("Connect")
            self._connect_button.setStyleSheet("""
                QPushButton {
                    background-color: #0078d4;
                    color: white;
                    border: none;
                    padding: 5px 15px;
                    border-radius: 3px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #106ebe;
                }
                QPushButton:pressed {
                    background-color: #0e5a9a;
                }
            """)
        else:
            if self._client.connect():
                self._connected = True
                self._client.set_message_callback(self.add_message)
                self._connect_button.setText("Disconnect")
                self._connect_button.setStyleSheet("""
                    QPushButton {
                        background-color: #d83b01;
                        color: white;
                        border: none;
                        padding: 5px 15px;
                        border-radius: 3px;
                        font-weight: bold;
                    }
                    QPushButton:hover {
                        background-color: #a7260a;
                    }
                    QPushButton:pressed {
                        background-color: #841f06;
                    }
                """)
