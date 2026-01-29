"""Debug window widget for displaying STM32 debug messages."""

from __future__ import annotations

from datetime import datetime
from typing import Any, Callable, Optional

from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ...ui.theme import set_role, set_variant


def _connect(signal: Any, slot: Callable[..., object]) -> None:
    getattr(signal, "connect")(slot)  # type: ignore


class DebugWindow(QWidget):
    """Widget for displaying real-time STM32 debug messages."""

    message_received = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._max_lines = 1000
        self._is_paused = False
        self._message_count = 0
        self._client = None
        self._connected = False

        _connect(self.message_received, self._append_message)
        self._setup_ui()
        self._setup_timer()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        layout.addWidget(self._create_header())

        self._text_display = QTextEdit()
        self._text_display.setReadOnly(True)
        set_role(self._text_display, "terminal")
        layout.addWidget(self._text_display)

        self._status_label = QLabel("Ready")
        set_role(self._status_label, "muted")
        layout.addWidget(self._status_label)

    def _create_header(self) -> QFrame:
        header = QFrame()
        set_role(header, "card")

        layout = QHBoxLayout(header)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(10)

        title = QLabel("STM32 Debug Output")
        set_role(title, "fieldLabel")
        layout.addWidget(title)

        layout.addStretch(1)

        self._connect_button = QPushButton("Connect")
        set_variant(self._connect_button, "primary")
        _connect(self._connect_button.clicked, self._toggle_connection)
        layout.addWidget(self._connect_button)

        self._pause_button = QPushButton("Pause")
        self._pause_button.setCheckable(True)
        _connect(self._pause_button.clicked, self._toggle_pause)
        layout.addWidget(self._pause_button)

        clear_button = QPushButton("Clear")
        _connect(clear_button.clicked, self._clear_display)
        layout.addWidget(clear_button)

        return header

    def _setup_timer(self) -> None:
        self._status_timer = QTimer()
        _connect(self._status_timer.timeout, self._update_status)
        self._status_timer.start(1000)

    def set_client(self, client) -> None:
        """Set the Ethernet debug client."""

        self._client = client

    def add_message(self, message: str) -> None:
        """Add a new debug message (thread-safe)."""

        if self._is_paused:
            return

        self._message_count += 1
        self.message_received.emit(message)

    def _append_message(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted = f"[{timestamp}] {message}"

        lower = message.lower()
        if any(w in lower for w in ["error", "failed", "fault"]):
            color = "#ef4444"
        elif any(w in lower for w in ["warning", "warn"]):
            color = "#f59e0b"
        elif any(w in lower for w in ["info", "status"]):
            color = "#60a5fa"
        else:
            color = "#e5e7eb"

        self._text_display.append(f'<span style="color: {color};">{formatted}</span>')

        if self._text_display.document().blockCount() > self._max_lines:
            cursor = self._text_display.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _toggle_pause(self, checked: bool) -> None:
        self._is_paused = checked
        self._pause_button.setText("Resume" if checked else "Pause")
        set_variant(self._pause_button, "warning" if checked else None)

    def _clear_display(self) -> None:
        self._text_display.clear()
        self._message_count = 0

    def _update_status(self) -> None:
        if self._is_paused:
            status = "Paused"
        elif self._connected:
            status = "Connected"
        else:
            status = "Disconnected"

        lines = self._text_display.document().blockCount()
        self._status_label.setText(
            f"{status} | Messages: {self._message_count} | Lines: {lines}"
        )

    def _toggle_connection(self) -> None:
        if not self._client:
            return

        if self._connected:
            self._client.disconnect()
            self._connected = False
            self._connect_button.setText("Connect")
            set_variant(self._connect_button, "primary")
            return

        if self._client.connect():
            self._connected = True
            self._client.set_message_callback(self.add_message)
            self._connect_button.setText("Disconnect")
            set_variant(self._connect_button, "danger")
