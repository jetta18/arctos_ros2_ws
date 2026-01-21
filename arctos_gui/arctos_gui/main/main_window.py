"""Main window for Arctos GUI."""

from PyQt5.QtWidgets import (
    QMainWindow,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


class ArctosMainWindow(QMainWindow):
    """Main window for the Arctos robot control GUI."""

    def __init__(self, jog_client, mks_config_client=None) -> None:
        super().__init__()
        self._jog_client = jog_client
        # mks_config_client is no longer needed - using direct CAN
        self._setup_ui()
        self._apply_modern_style()

    def _setup_ui(self) -> None:
        """Setup the user interface."""
        self.setWindowTitle("Arctos Control GUI")
        self.setGeometry(100, 100, 900, 700)

        # Create central widget with tab layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # Create tab widget
        self._tab_widget = QTabWidget()
        self._tab_widget.setStyleSheet("""
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

        # Add jog tab
        from ..components.jog import JogWidget
        self._jog_widget = JogWidget(self._jog_client)
        self._tab_widget.addTab(self._jog_widget, "Jog Control")

        # Add MKS config tab
        from ..components.mks_config import MKSConfigWidget
        self._mks_config_widget = MKSConfigWidget()
        self._tab_widget.addTab(self._mks_config_widget, "MKS Motor Config")

        layout.addWidget(self._tab_widget)

    def _apply_modern_style(self) -> None:
        """Apply modern styling to the main window."""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
        """)
