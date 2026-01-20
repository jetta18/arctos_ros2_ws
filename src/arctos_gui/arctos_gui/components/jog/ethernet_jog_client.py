"""Ethernet jog client for STM32 control."""

import socket
import struct
import math
import threading
import time
from typing import Optional


# Protocol constants
STEPPER_PROTOCOL_VERSION = 1
CMD_MOVE_SINGLE = 0x03
CMD_GET_STATE = 0x10
RESP_OK = 0x00
RESP_STATE = 0x02

# Motor configuration (matching STM32)
STEPS_PER_REV = 200
MICROSTEPS = 16
GEAR_RATIOS = [13.5, 150.0, 150.0, 48.0, 27.3375, 10.0]  # X, Y, Z, A, B, C


class EthernetJogClient:
    """Ethernet client for sending jog commands to STM32."""
    
    def __init__(self, host: str = "192.168.178.159", port: int = 8888):
        """Initialize the Ethernet jog client.
        
        Args:
            host: STM32 IP address
            port: STM32 port number
        """
        self.host = host
        self.port = port
        self._socket: Optional[socket.socket] = None
        self._sequence = 0
        self._connected = False
        self._reconnect_thread: Optional[threading.Thread] = None
        self._stop_reconnect = threading.Event()
        self._connection_lock = threading.Lock()
        
        # Calculate steps per revolution for each axis
        self.steps_per_rev = [STEPS_PER_REV * MICROSTEPS * gr for gr in GEAR_RATIOS]
        
        # Track current positions in steps
        self.current_positions = [0.0] * 6
        
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
            
            # Start reconnection monitoring
            self._stop_reconnect.clear()
            if self._reconnect_thread is None or not self._reconnect_thread.is_alive():
                self._reconnect_thread = threading.Thread(target=self._reconnection_monitor, daemon=True)
                self._reconnect_thread.start()
            
            # Fetch initial positions
            self.update_positions()
            
            print(f"Successfully connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            self._cleanup_connection()
            return False
            
    def disconnect(self) -> None:
        """Disconnect from the STM32."""
        with self._connection_lock:
            self._stop_reconnect.set()
            if self._reconnect_thread and self._reconnect_thread.is_alive():
                self._reconnect_thread.join(timeout=1.0)
            self._cleanup_connection()
        
    def _cleanup_connection(self) -> None:
        """Clean up connection resources."""
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None
        self._connected = False
        
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
            print(f"Send command failed: {e}")
            with self._connection_lock:
                self._cleanup_connection()
            return None
            
    def rad_to_steps(self, radians: float, axis: int) -> float:
        """Convert radians to steps for a specific axis.
        
        Args:
            radians: Angle in radians
            axis: Axis index [0..5]
            
        Returns:
            Position in steps
        """
        return radians * (self.steps_per_rev[axis] / (2.0 * math.pi))
        
    def send_jog(self, axis_index: int, delta_rad: float, velocity_rad_s: float) -> None:
        """Send a jog command to the STM32.
        
        Args:
            axis_index: Joint index [0..5]
            delta_rad: Relative position delta in radians
            velocity_rad_s: Jog velocity in radians per second
        """
        if not self._connected or axis_index < 0 or axis_index >= 6:
            print(f"Cannot send jog: connected={self._connected}, axis={axis_index}")
            return
            
        # Convert delta to steps
        delta_steps = self.rad_to_steps(delta_rad, axis_index)
        
        # Calculate new absolute position
        new_position_steps = self.current_positions[axis_index] + delta_steps
        
        # Convert velocity to steps/s
        velocity_steps = self.rad_to_steps(abs(velocity_rad_s), axis_index)
        
        print(f"Jog axis {axis_index}: delta={delta_rad:.3f}rad ({delta_steps:.1f}steps), "
              f"new_pos={new_position_steps:.1f}steps, vel={velocity_rad_s:.3f}rad/s ({velocity_steps:.1f}steps/s)")
        
        # Create payload for single axis move
        # Payload format: axis_index (uint8), target_position (float), velocity (float), acceleration (float)
        payload = struct.pack('<Bfff', axis_index, new_position_steps, velocity_steps, 50.0)
        
        response = self._send_command(CMD_MOVE_SINGLE, payload)
        
        if response and response[0] == RESP_OK:
            # Update our tracked position
            self.current_positions[axis_index] = new_position_steps
            print(f"Jog command succeeded for axis {axis_index}")
        else:
            print(f"Jog command failed for axis {axis_index}, response={response}")
            # Try to refresh positions on failure
            self.update_positions()
            
    def _reconnection_monitor(self) -> None:
        """Background thread that monitors connection and attempts reconnection."""
        while not self._stop_reconnect.is_set():
            time.sleep(1.0)  # Check every second
            
            if self._stop_reconnect.is_set():
                break
                
            # Check if connection is still valid
            if self._connected and self._socket:
                try:
                    # Simple health check - try to get state
                    response = self._send_command(CMD_GET_STATE)
                    if response is None:
                        print("Connection lost, attempting reconnection...")
                        with self._connection_lock:
                            self._cleanup_connection()
                except:
                    with self._connection_lock:
                        self._cleanup_connection()
            
            # Attempt reconnection if disconnected
            if not self._connected and not self._stop_reconnect.is_set():
                print("Attempting to reconnect...")
                if self._connect_internal():
                    print("Reconnection successful")
                else:
                    print("Reconnection failed, will retry in 1 second")
                    
    def __del__(self):
        """Cleanup when object is destroyed."""
        self.disconnect()
            
    def update_positions(self) -> None:
        """Update current positions from STM32."""
        if not self._connected:
            return
            
        # Use GET_STATE to read current positions
        response = self._send_command(CMD_GET_STATE)
        if response and response[0] == RESP_STATE:
            positions = struct.unpack('<6f', response[1:25])
            self.current_positions = list(positions)
