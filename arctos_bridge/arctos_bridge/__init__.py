"""Arctos Bridge — ROS2 to STM32 communication bridge.

This package provides a bidirectional communication bridge between the Arctos
STM32 firmware and the ROS2 ecosystem.

Main components:
  - arctos_bridge_node: Main ROS2 node
  - core: Parameter management, publishers, subscribers
  - services: Service handlers and service manager
  - communication: Broadcast receiver and state publisher
  - protocol: STM32 protocol client and unit conversion
  - actions: Trajectory action server
"""

__version__ = "2.0.0"

from .arctos_bridge_node import ArctossBridgeNode, main

__all__ = [
    "ArctossBridgeNode",
    "main",
]
