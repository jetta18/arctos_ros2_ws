"""Ethernet client for STM32 debug messages."""

import logging
import socket
import struct
import threading
from typing import Callable, Optional

_LOGGER = logging.getLogger(__name__)


# Protocol constants
STEPPER_PROTOCOL_VERSION = 1
CMD_GET_STATE = 0x10
RESP_STATE = 0x02


class EthernetDebugClient:
    """Ethernet client for receiving debug messages from STM32."""
    
    def __init__(self, host: str = "192.168.178.159", port: int = 8888):
        """Initialize the Ethernet debug client.
        
        Args:
            host: STM32 IP address
            port: STM32 port number
        """
        self.host = host
        self.port = port
        self._socket: Optional[socket.socket] = None
        self._sequence = 0
        self._connected = False
        self._message_callback: Optional[Callable[[str], None]] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reconnect_thread: Optional[threading.Thread] = None
        self._connection_lock = threading.Lock()
        
    def set_message_callback(self, callback: Callable[[str], None]) -> None:
        """Set the callback function for received messages.
        
        Args:
            callback: Function to call when a debug message is received
        """
        self._message_callback = callback
        
    def connect(self) -> bool:
        """Connect to the STM32.
        
        Returns:
            True if connection successful, False otherwise
        """
        with self._connection_lock:
            return self._connect_internal()
            
    def _connect_internal(self) -> bool:
        """Internal connection method without locking."""
        try:
            # Clean up existing socket if any
            if self._socket:
                self._socket.close()
                self._socket = None
            
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(2.0)
            self._socket.connect((self.host, self.port))
            self._connected = True
            
            # Start monitoring thread
            self._stop_event.clear()
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
                self._thread.start()
            
            # Start reconnection monitoring
            if self._reconnect_thread is None or not self._reconnect_thread.is_alive():
                self._reconnect_thread = threading.Thread(target=self._reconnection_monitor, daemon=True)
                self._reconnect_thread.start()
            
            _LOGGER.info("Connected to %s:%s", self.host, self.port)
            return True
        except Exception as e:
            _LOGGER.warning("Failed to connect to %s:%s: %s", self.host, self.port, e)
            self._cleanup_connection()
            return False
            
    def disconnect(self) -> None:
        """Disconnect from the STM32."""
        with self._connection_lock:
            self._stop_event.set()
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
            self._cleanup_connection()
        
    def is_connected(self) -> bool:
        """Check if connected to STM32.
        
        Returns:
            True if connected, False otherwise
        """
        return self._connected
        
    def _send_command(self, command: int, payload: bytes = b'') -> Optional[bytes]:
        """Send a command to the STM32.
        
        Args:
            command: Command byte
            payload: Command payload
            
        Returns:
            Response data or None if failed
        """
        if not self._connected or not self._socket:
            return None
            
        self._sequence += 1
        header = struct.pack('<BBHI', STEPPER_PROTOCOL_VERSION, command, len(payload), self._sequence)
        
        try:
            self._socket.sendall(header + payload)
            resp_header = self._socket.recv(8)
            if len(resp_header) < 8:
                return None
                
            version, response, length, seq = struct.unpack('<BBHI', resp_header)
            payload_data = b''
            if length > 0:
                payload_data = self._socket.recv(length)
                
            return bytes([response]) + payload_data
        except Exception as e:
            _LOGGER.debug("Send command failed: %s", e)
            with self._connection_lock:
                self._cleanup_connection()
            return None
            
    def _monitor_loop(self) -> None:
        """Main monitoring loop that runs in a separate thread."""
        import time
        
        while not self._stop_event.is_set() and self._connected:
            try:
                # Get current state from STM32
                response = self._send_command(CMD_GET_STATE)
                if response and response[0] == RESP_STATE:
                    # Parse state data
                    positions = struct.unpack('<6f', response[1:25])
                    velocities = struct.unpack('<6f', response[25:49])
                    
                    # Create debug message
                    msg = f"State: Pos=[{positions[0]:.1f}, {positions[1]:.1f}, {positions[2]:.1f}] " \
                          f"Vel=[{velocities[0]:.2f}, {velocities[1]:.2f}, {velocities[2]:.2f}]"
                    
                    if self._message_callback:
                        self._message_callback(msg)
                        
                time.sleep(0.1)  # Update at 10 Hz
                
            except Exception as e:
                _LOGGER.debug("Monitor loop error: %s", e)
                with self._connection_lock:
                    self._cleanup_connection()
                break
                
    def _cleanup_connection(self) -> None:
        """Clean up connection resources."""
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
        self._connected = False
        
    def _reconnection_monitor(self) -> None:
        """Background thread that monitors connection and attempts reconnection."""
        import time
        
        while not self._stop_event.is_set():
            time.sleep(1.0)  # Check every second
            
            if self._stop_event.is_set():
                break
                
            # Check if connection is still valid
            if self._connected and self._socket:
                try:
                    # Simple health check - try to get state
                    response = self._send_command(CMD_GET_STATE)
                    if response is None:
                        _LOGGER.info("Connection lost, attempting reconnection")
                        with self._connection_lock:
                            self._cleanup_connection()
                except Exception:
                    with self._connection_lock:
                        self._cleanup_connection()
            
            # Attempt reconnection if disconnected
            if not self._connected and not self._stop_event.is_set():
                _LOGGER.info("Attempting to reconnect")
                if self._connect_internal():
                    _LOGGER.info("Reconnection successful")
                else:
                    _LOGGER.info("Reconnection failed; retrying")
                    
    def __del__(self):
        """Cleanup when object is destroyed."""
        self.disconnect()
