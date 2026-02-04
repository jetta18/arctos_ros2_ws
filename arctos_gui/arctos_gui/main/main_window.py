"""Main window for Arctos GUI."""

from PyQt5.QtGui import QCloseEvent, QGuiApplication, QShowEvent
from PyQt5.QtWidgets import (
    QMainWindow,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


class ArctosMainWindow(QMainWindow):
    """Main window for the Arctos robot control GUI."""

    def __init__(self, jog_client, cartesian_jog_client=None, mks_config_client=None) -> None:
        super().__init__()
        self._initial_geometry_applied = False
        self._jog_client = jog_client
        self._cartesian_jog_client = cartesian_jog_client
        # mks_config_client is no longer needed - using direct CAN
        self._setup_ui()

    def showEvent(self, a0: QShowEvent) -> None:  # noqa: N802 (Qt override)
        super().showEvent(a0)
        if self._initial_geometry_applied:
            return
        self._apply_initial_window_geometry()
        self._initial_geometry_applied = True

    def closeEvent(self, a0: QCloseEvent) -> None:  # noqa: N802 (Qt override)
        if hasattr(self, "_mks_config_widget") and self._mks_config_widget is not None:
            try:
                self._mks_config_widget.shutdown()
            except Exception:
                pass

        super().closeEvent(a0)

    def _setup_ui(self) -> None:
        """Setup the user interface."""
        self.setWindowTitle("Arctos Control GUI")
        # Avoid hard-coded window sizes that can exceed smaller screens.
        # Geometry is applied on first show() based on available screen space.

        # Keep a small baseline minimum so the layout stays usable.
        self.setMinimumSize(640, 480)

        # Create central widget with tab layout
        central_widget = QWidget()
        central_widget.setObjectName("appCentral")
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        # Create tab widget
        self._tab_widget = QTabWidget()
        self._tab_widget.setObjectName("mainTabs")

        # Add jog tab
        from ..components.jog import JogWidget
        self._jog_widget = JogWidget(self._jog_client)
        self._tab_widget.addTab(self._jog_widget, "Joint Jog")

        # Add cartesian jog tab (always visible, connection handled in widget)
        from ..components.cartesian_jog import CartesianJogWidget
        self._cartesian_jog_widget = CartesianJogWidget(self._cartesian_jog_client)
        self._tab_widget.addTab(self._cartesian_jog_widget, "Cartesian Jog")

        # Add MKS config tab
        from ..components.mks_config import MKSConfigWidget
        self._mks_config_widget = MKSConfigWidget()
        self._tab_widget.addTab(self._mks_config_widget, "MKS Motor Config")

        layout.addWidget(self._tab_widget)

    def _apply_initial_window_geometry(self) -> None:
        """Size and center the window to fit the current screen."""
        preferred_w, preferred_h = 900, 700

        screen = self.screen() or QGuiApplication.primaryScreen()
        if screen is None:
            self.resize(preferred_w, preferred_h)
            return

        available = screen.availableGeometry()
        max_w = int(available.width() * 0.95)
        max_h = int(available.height() * 0.95)

        w = min(preferred_w, max_w, available.width())
        h = min(preferred_h, max_h, available.height())

        # Keep a sensible minimum, but never exceed the available area.
        w = min(max(640, w), available.width())
        h = min(max(480, h), available.height())

        self.resize(w, h)
        self.move(
            available.x() + (available.width() - w) // 2,
            available.y() + (available.height() - h) // 2,
        )
