"""Controller switching utilities for arctos_servo."""

from __future__ import annotations

import logging
import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class ControllerSwitchConfig:
    """Configuration for controller switching commands."""

    servo_controller: str
    trajectory_controller: str
    controller_manager: str = '/controller_manager'
    timeout_sec: float = 10.0


class ControllerSwitcher:
    """Load and switch between trajectory and servo controllers."""

    def __init__(
        self,
        config: ControllerSwitchConfig,
        logger: logging.Logger | None = None,
    ) -> None:
        self._config = config
        self._logger = logger or logging.getLogger(__name__)

    def ensure_servo_loaded(self) -> None:
        """Ensure the servo controller is loaded (inactive)."""
        self._run_command(
            [
                'ros2', 'run', 'controller_manager', 'spawner',
                self._config.servo_controller, '--inactive',
                '-c', self._config.controller_manager,
            ],
            check=False,
            log_prefix='Load servo controller',
        )

    def switch_to_servo(self) -> bool:
        """Deactivate trajectory controller and activate servo controller."""
        return self._run_command(
            [
                'ros2', 'control', 'switch_controllers',
                '--deactivate', self._config.trajectory_controller,
                '--activate', self._config.servo_controller,
            ],
            check=True,
            log_prefix='Switch to servo controller',
        )

    def switch_to_trajectory(self) -> bool:
        """Deactivate servo controller and activate trajectory controller."""
        return self._run_command(
            [
                'ros2', 'control', 'switch_controllers',
                '--deactivate', self._config.servo_controller,
                '--activate', self._config.trajectory_controller,
            ],
            check=False,
            log_prefix='Switch to trajectory controller',
        )

    def _run_command(
        self,
        command: list[str],
        check: bool,
        log_prefix: str,
    ) -> bool:
        try:
            subprocess.run(
                command,
                capture_output=True,
                timeout=self._config.timeout_sec,
                check=check,
            )
            return True
        except subprocess.CalledProcessError as exc:
            self._logger.warning('%s failed: %s', log_prefix, exc)
            return False
        except Exception as exc:
            self._logger.debug('%s error: %s', log_prefix, exc)
            return False
