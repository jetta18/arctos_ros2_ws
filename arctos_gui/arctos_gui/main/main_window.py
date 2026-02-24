"""Main window for Arctos GUI."""

from __future__ import annotations

from typing import Optional

from PyQt5.QtGui import QCloseEvent, QGuiApplication, QShowEvent
from PyQt5.QtWidgets import (
    QMainWindow,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..backend.settings_manager import SettingsManager
from ..components.homing import HomingWidget
from ..components.homing.ros_homing_client import ArctosRosHomingClient
from ..components.jog import JogWidget
from ..components.motor_settings import MotorSettingsWidget
from ..components.motor_settings.ros_motor_settings_client import (
    ArctosMotorSettingsClient,
)
from ..components.robot_status import RobotStatusWidget
from ..components.robot_status.ros_robot_status_client import (
    ArctosRosRobotStatusClient,
)
from ..components.servo_jog import ServoJogWidget
from ..ui.theme import CARD_SPACING, OUTER_MARGIN


class ArctosMainWindow(QMainWindow):
    """Main window for the Arctos robot control GUI."""

    def __init__(
        self,
        jog_client,
        servo_jog_client=None,
        settings_manager: Optional[SettingsManager] = None,
    ) -> None:
        """Initializes the main window.

        Args:
            jog_client: Joint jog ROS client.
            servo_jog_client: Servo jog ROS client (optional).
            settings_manager: Shared settings persistence backend.
        """
        super().__init__()
        self._initial_geometry_applied = False
        self._jog_client = jog_client
        self._servo_jog_client = servo_jog_client
        self._settings_manager = settings_manager or SettingsManager()
        self._setup_ui()

    def showEvent(self, a0: QShowEvent) -> None:  # noqa: N802 (Qt override)
        """Handle the Qt show event."""
        super().showEvent(a0)
        if self._initial_geometry_applied:
            return
        self._apply_initial_window_geometry()
        self._initial_geometry_applied = True

    def closeEvent(self, a0: QCloseEvent) -> None:  # noqa: N802 (Qt override)
        """Handle the Qt close event."""
        super().closeEvent(a0)

    def _setup_ui(self) -> None:
        """Setup the user interface."""
        self.setWindowTitle("Arctos Control GUI")
        self.setMinimumSize(640, 480)

        central_widget = QWidget()
        central_widget.setObjectName("appCentral")
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN, OUTER_MARGIN)
        layout.setSpacing(CARD_SPACING)

        self._tab_widget = QTabWidget()
        self._tab_widget.setObjectName("mainTabs")

        self._jog_widget = JogWidget(self._jog_client)
        self._tab_widget.addTab(self._jog_widget, "Joint Jog")

        self._servo_jog_widget = ServoJogWidget(self._servo_jog_client)
        self._tab_widget.addTab(self._servo_jog_widget, "Servo Jog")

        self._robot_status_client = ArctosRosRobotStatusClient()
        self._robot_status_widget = RobotStatusWidget(client=self._robot_status_client)
        self._tab_widget.addTab(self._robot_status_widget, "Robot Status")

        motor_client = ArctosMotorSettingsClient()
        self._motor_settings_widget = MotorSettingsWidget(
            settings_manager=self._settings_manager,
            client=motor_client,
        )
        self._tab_widget.addTab(self._motor_settings_widget, "Motor Settings")

        homing_client = ArctosRosHomingClient()
        self._homing_widget = HomingWidget(
            client=homing_client,
            settings_manager=self._settings_manager,
        )
        self._tab_widget.addTab(self._homing_widget, "Homing")

        layout.addWidget(self._tab_widget)

    def _apply_initial_window_geometry(self) -> None:
        """Maximise the window on first show."""
        self.showMaximized()
