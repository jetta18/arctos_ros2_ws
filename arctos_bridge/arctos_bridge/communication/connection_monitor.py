"""Connection monitor for the STM32 link.

Tracks broadcast reception, detects connection loss, and drives
automatic reconnection with exponential backoff.

Design constraints:
  - Must never block the command socket during normal operation.
  - Status is published periodically (1 Hz) so late-subscribing GUI
    nodes always receive current state.
  - Reconnection runs in a short-lived daemon thread to keep the
    ROS timer callback non-blocking.
"""

import threading
import time
from typing import Callable

from rclpy.node import Node

from arctos_msgs.msg import ConnectionStatus

_STATUS_PUBLISH_PERIOD_S = 1.0


class ConnectionMonitor:
    """Lightweight STM32 connection monitor.

    Call ``notify_received()`` from the broadcast callback on every
    incoming packet.  A 1 Hz ROS timer (``_on_timer``) checks elapsed
    time, publishes status, and spawns a reconnect thread when needed.

    Attributes:
        status: Current connection status enum value.
        reconnect_attempts: Cumulative reconnection attempts since last connect.
    """

    def __init__(
        self,
        node: Node,
        timeout_s: float,
        initial_delay_s: float,
        max_delay_s: float,
        backoff_multiplier: float,
        auto_reconnect: bool,
        reconnect_fn: Callable[[], bool],
        status_publish_fn: Callable[[ConnectionStatus], None],
    ) -> None:
        self._node = node
        self._timeout_s = timeout_s
        self._initial_delay_s = initial_delay_s
        self._max_delay_s = max_delay_s
        self._backoff_multiplier = backoff_multiplier
        self._auto_reconnect = auto_reconnect
        self._reconnect_fn = reconnect_fn
        self._status_publish_fn = status_publish_fn

        self._lock = threading.Lock()
        self._last_received: float = 0.0
        self._status: int = ConnectionStatus.DISCONNECTED
        self._reconnect_attempts: int = 0
        self._current_delay: float = initial_delay_s
        self._reconnect_thread: threading.Thread | None = None
        self._last_reconnect_time: float = 0.0

        self._timer = node.create_timer(
            _STATUS_PUBLISH_PERIOD_S, self._on_timer,
        )

    @property
    def status(self) -> int:
        with self._lock:
            return self._status

    @property
    def reconnect_attempts(self) -> int:
        with self._lock:
            return self._reconnect_attempts

    @property
    def last_received_time(self) -> float:
        with self._lock:
            return self._last_received

    def notify_received(self) -> None:
        """Called from the broadcast RX thread on every received packet."""
        now = time.time()
        with self._lock:
            self._last_received = now
            was_disconnected = self._status != ConnectionStatus.CONNECTED
            self._status = ConnectionStatus.CONNECTED
            if was_disconnected:
                self._reconnect_attempts = 0
                self._current_delay = self._initial_delay_s

        if was_disconnected:
            self._node.get_logger().info("STM32 connection established")
            self._publish_status("Connected")

    def force_reconnect(self) -> bool:
        """Trigger a single immediate reconnection attempt (blocking).

        Called from the Reconnect service handler thread, which is
        separate from the broadcast RX and timer threads.
        """
        self._node.get_logger().info("Manual reconnection requested")
        return self._do_single_reconnect()

    def shutdown(self) -> None:
        """Cancel the timer. Safe to call multiple times."""
        self._timer.cancel()

    def _on_timer(self) -> None:
        """1 Hz callback: publish status and trigger reconnect if needed."""
        with self._lock:
            last = self._last_received
            current = self._status

        self._publish_status(self._status_message(current))

        if last == 0.0:
            return

        elapsed = time.time() - last
        timed_out = elapsed > self._timeout_s

        if current == ConnectionStatus.CONNECTED and timed_out:
            with self._lock:
                self._status = ConnectionStatus.DISCONNECTED
                self._current_delay = self._initial_delay_s
            self._node.get_logger().warn(
                f"STM32 connection lost (no broadcast for {elapsed:.1f}s)"
            )
            self._publish_status("Connection lost")
            if self._auto_reconnect:
                self._maybe_spawn_reconnect()

        elif current != ConnectionStatus.CONNECTED and self._auto_reconnect:
            self._maybe_spawn_reconnect()

    def _maybe_spawn_reconnect(self) -> None:
        """Spawn a reconnect thread if none is running and backoff allows."""
        if self._reconnect_thread is not None and self._reconnect_thread.is_alive():
            return

        now = time.time()
        with self._lock:
            delay = self._current_delay
            last_try = self._last_reconnect_time

        if now - last_try < delay:
            return

        with self._lock:
            self._status = ConnectionStatus.RECONNECTING
        self._publish_status("Reconnecting...")

        self._reconnect_thread = threading.Thread(
            target=self._reconnect_thread_fn,
            daemon=True,
            name="ReconnectWorker",
        )
        self._reconnect_thread.start()

    def _reconnect_thread_fn(self) -> None:
        """Runs a single reconnect attempt in a daemon thread."""
        self._do_single_reconnect()

    def _do_single_reconnect(self) -> bool:
        """Execute one ping + subscribe attempt.

        Updates backoff state regardless of outcome.
        """
        with self._lock:
            self._reconnect_attempts += 1
            attempt = self._reconnect_attempts
            self._last_reconnect_time = time.time()

        self._node.get_logger().info(f"Reconnection attempt #{attempt}")

        try:
            success = self._reconnect_fn()
        except Exception as e:
            self._node.get_logger().warn(f"Reconnection failed: {e}")
            success = False

        if success:
            with self._lock:
                self._status = ConnectionStatus.CONNECTED
                self._last_received = time.time()
                self._reconnect_attempts = 0
                self._current_delay = self._initial_delay_s
            self._node.get_logger().info("Reconnection successful")
            self._publish_status("Reconnected")
            return True

        with self._lock:
            self._status = ConnectionStatus.RECONNECTING
            self._current_delay = min(
                self._current_delay * self._backoff_multiplier,
                self._max_delay_s,
            )
        self._publish_status(f"Reconnect attempt #{attempt} failed")
        return False

    def _publish_status(self, message: str) -> None:
        msg = ConnectionStatus()
        with self._lock:
            msg.status = self._status
            msg.last_received_time = self._last_received
            msg.reconnect_attempts = self._reconnect_attempts
        msg.message = message
        self._status_publish_fn(msg)

    @staticmethod
    def _status_message(status: int) -> str:
        if status == ConnectionStatus.CONNECTED:
            return "Connected"
        if status == ConnectionStatus.RECONNECTING:
            return "Reconnecting..."
        return "Disconnected"
