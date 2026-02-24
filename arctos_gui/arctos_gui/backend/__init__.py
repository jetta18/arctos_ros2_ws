"""Backend package for Arctos GUI.

Provides persistence (SettingsManager) and shared runtime state (AppState).
No PyQt5 or rclpy imports are allowed in this package.
"""

from __future__ import annotations

from .app_state import AppState
from .settings_manager import SettingsManager

__all__ = ["AppState", "SettingsManager"]
