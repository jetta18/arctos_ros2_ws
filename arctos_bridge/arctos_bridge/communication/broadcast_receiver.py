"""UDP broadcast receiver for STM32 state broadcasts.

This module handles receiving and parsing state broadcast packets from the STM32
over UDP. It runs in a separate thread and forwards parsed state data to a callback.
"""

import socket
import threading
from typing import Callable, Optional, Dict, Any
from rclpy.node import Node

from arctos_bridge.protocol.stm32_protocol import (
    STM32CommandClient,
    parse_broadcast,
)


class BroadcastReceiver:
    """Receives and parses UDP state broadcasts from the STM32.
    
    Runs a background thread that listens for UDP packets on the configured port,
    parses them, and forwards the parsed state to a callback function.
    
    Attributes:
        listen_port: Local UDP port to bind to
        broadcast_rate_hz: Requested broadcast rate from STM32
    """
    
    def __init__(
        self,
        node: Node,
        cmd_client: STM32CommandClient,
        listen_port: int,
        broadcast_rate_hz: int,
        state_callback: Callable[[Dict[str, Any]], None],
    ) -> None:
        """Initialize the broadcast receiver.
        
        Args:
            node: ROS2 node for logging
            cmd_client: STM32 command client for subscription commands
            listen_port: Local UDP port to receive broadcasts on
            broadcast_rate_hz: Requested broadcast rate (1-200 Hz)
            state_callback: Callback function to receive parsed state dicts
        """
        self._node = node
        self._cmd_client = cmd_client
        self._listen_port = listen_port
        self._broadcast_rate_hz = broadcast_rate_hz
        self._state_callback = state_callback
        
        self._listen_sock: Optional[socket.socket] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_thread_running = False
        
        self._init_socket()
        self._start_receiver_thread()
        self._subscribe_to_stm32()
    
    def _init_socket(self) -> None:
        """Initialize and bind the UDP socket for receiving broadcasts."""
        self._listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen_sock.bind(("0.0.0.0", self._listen_port))
        self._listen_sock.settimeout(1.0)
        self._node.get_logger().info(f"Broadcast socket bound to port {self._listen_port}")
    
    def _start_receiver_thread(self) -> None:
        """Start the background thread for receiving broadcasts."""
        self._rx_thread_running = True
        self._rx_thread = threading.Thread(
            target=self._broadcast_rx_loop,
            daemon=True,
            name="BroadcastReceiver",
        )
        self._rx_thread.start()
        self._node.get_logger().info("Broadcast receiver thread started")
    
    def _subscribe_to_stm32(self) -> None:
        """Send subscription command to STM32 to start broadcasts."""
        try:
            success = self._cmd_client.subscribe(
                self._listen_port, self._broadcast_rate_hz
            )
            if success:
                self._node.get_logger().info(
                    f"Subscribed to state broadcasts "
                    f"(port={self._listen_port}, rate={self._broadcast_rate_hz} Hz)"
                )
            else:
                self._node.get_logger().error("Failed to subscribe to state broadcasts")
        except Exception as e:
            self._node.get_logger().error(f"Subscribe failed: {e}")
    
    def _broadcast_rx_loop(self) -> None:
        """Main loop for receiving and parsing broadcast packets.
        
        Runs in a separate thread. Receives UDP packets, parses them,
        and forwards valid state data to the callback.
        """
        while self._rx_thread_running:
            try:
                data, _ = self._listen_sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            
            state = parse_broadcast(data)
            if state is None:
                continue
            
            self._state_callback(state)
    
    def shutdown(self) -> None:
        """Stop the receiver thread and close the socket.
        
        Should be called during node shutdown to cleanly stop the background thread.
        """
        self._node.get_logger().info("Shutting down broadcast receiver")
        self._rx_thread_running = False
        
        try:
            self._cmd_client.unsubscribe()
        except Exception:
            pass
        
        try:
            if self._listen_sock is not None:
                self._listen_sock.close()
        except Exception:
            pass
        
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
