"""Settings persistence for Arctos GUI.

Stores and retrieves application settings as YAML in the package's
config directory (share/arctos_gui/config/settings.yaml).
"""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Any

import yaml

logger = logging.getLogger(__name__)


def _default_settings_file() -> Path:
    """Returns the path to the package-local settings file.

    Resolves the installed share directory via ament_index. Falls back to
    a path relative to this source file when running outside a ROS install.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory("arctos_gui"))
    except Exception:
        share_dir = Path(__file__).parents[3] / "share" / "arctos_gui"
    return share_dir / "config" / "settings.yaml"


_SETTINGS_FILE = _default_settings_file()


class SettingsManager:
    """Manages persistent application settings stored as YAML.

    All read and write access to the settings file must go through this class.
    Thread-safe via an internal lock.

    Example:
        manager = SettingsManager()
        manager.load()
        manager.set("motor.X.can_id", 1)
        manager.save()
        value = manager.get("motor.X.can_id", default=1)
    """

    def __init__(self, settings_file: Path = _SETTINGS_FILE) -> None:
        """Initializes the SettingsManager.

        Args:
            settings_file: Path to the YAML settings file.
        """
        self._settings_file = settings_file
        self._data: dict[str, Any] = {}
        self._lock = threading.Lock()

    def load(self) -> SettingsManager:
        """Loads settings from disk.

        If the file does not exist, starts with an empty settings dict.

        Returns:
            self, for chaining.
        """
        with self._lock:
            if self._settings_file.exists():
                try:
                    with open(self._settings_file, "r", encoding="utf-8") as fh:
                        loaded = yaml.safe_load(fh)
                        self._data = loaded if isinstance(loaded, dict) else {}
                    logger.info("Settings loaded from %s", self._settings_file)
                except Exception:
                    logger.exception("Failed to load settings from %s", self._settings_file)
                    self._data = {}
            else:
                self._data = {}
                logger.info("No settings file found at %s — using defaults", self._settings_file)
        return self

    def save(self) -> None:
        """Persists current settings to disk.

        Creates the config directory if it does not exist.
        """
        with self._lock:
            try:
                self._settings_file.parent.mkdir(parents=True, exist_ok=True)
                with open(self._settings_file, "w", encoding="utf-8") as fh:
                    yaml.safe_dump(self._data, fh, default_flow_style=False, allow_unicode=True)
                logger.info("Settings saved to %s", self._settings_file)
            except Exception:
                logger.exception("Failed to save settings to %s", self._settings_file)

    def get(self, key: str, default: Any = None) -> Any:
        """Returns the value for a dot-separated key.

        Args:
            key: Dot-separated key path, e.g. ``"motor.X.can_id"``.
            default: Value to return when the key is absent.

        Returns:
            The stored value, or *default* if not found.
        """
        with self._lock:
            return self._get_nested(self._data, key.split("."), default)

    def set(self, key: str, value: Any) -> None:
        """Stores a value under a dot-separated key.

        Intermediate dicts are created as needed.

        Args:
            key: Dot-separated key path, e.g. ``"motor.X.can_id"``.
            value: Value to store.
        """
        with self._lock:
            self._set_nested(self._data, key.split("."), value)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _get_nested(data: dict[str, Any], parts: list[str], default: Any) -> Any:
        """Traverses nested dicts following *parts* and returns the leaf value."""
        node: Any = data
        for part in parts:
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    @staticmethod
    def _set_nested(data: dict[str, Any], parts: list[str], value: Any) -> None:
        """Creates or updates a nested key path in *data*."""
        node = data
        for part in parts[:-1]:
            if part not in node or not isinstance(node[part], dict):
                node[part] = {}
            node = node[part]
        node[parts[-1]] = value
