#!/usr/bin/env python3
"""
STM32 Stepper Motor Control Client
Compatible with ROS2 Control Hardware Interface
"""

import socket
import struct
import time
import numpy as np
from typing import List, Tuple, Optional

# Protocol constants
STEPPER_PROTOCOL_VERSION = 1

# Command types
CMD_MOVE_LINEAR = 0x01
CMD_MOVE_RAPID = 0x02
CMD_MOVE_JOINT = 0x03
CMD_HOME = 0x04
CMD_STOP = 0x05
CMD_GET_POSITION = 0x06
CMD_SET_POSITION = 0x07
CMD_GET_STATUS = 0x08
CMD_SET_CONFIG = 0x09
CMD_ENABLE = 0x0A
CMD_DISABLE = 0x0B
CMD_CLEAR_BUFFER = 0x0C
CMD_JTC_STREAM = 0x0D
CMD_SET_DIRECT = 0x0E

# Response types
RESP_OK = 0x00
RESP_ERROR = 0x01
RESP_POSITION = 0x02
RESP_STATUS = 0x03
RESP_BUFFER_FULL = 0x04


class StepperClient:
    """Client for STM32 Stepper Motor Control"""
    
    def __init__(self, host: str = "192.168.178.159", port: int = 8888):
        self.host = host
        self.port = port
        self.socket = None
        self.sequence = 0
        
    def connect(self) -> bool:
        """Connect to STM32 stepper controller"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            print(f"✓ Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from controller"""
        if self.socket:
            self.socket.close()
            self.socket = None
            print("✓ Disconnected")
    
    def _pack_header(self, command: int, payload_length: int) -> bytes:
        """Pack command header"""
        self.sequence += 1
        return struct.pack('<BBHI', 
                          STEPPER_PROTOCOL_VERSION,
                          command,
                          payload_length,
                          self.sequence)
    
    def _unpack_header(self, data: bytes) -> Tuple[int, int, int, int]:
        """Unpack response header"""
        return struct.unpack('<BBHI', data[:8])
    
    def _send_command(self, command: int, payload: bytes = b'') -> Optional[bytes]:
        """Send command and receive response"""
        if not self.socket:
            print("✗ Not connected")
            return None
        
        try:
            # Pack and send
            header = self._pack_header(command, len(payload))
            packet = header + payload
            self.socket.sendall(packet)
            
            # Receive response header
            resp_header = self.socket.recv(8)
            if len(resp_header) < 8:
                print("✗ Invalid response header")
                return None
            
            version, response, length, seq = self._unpack_header(resp_header)
            
            # Receive payload
            payload_data = b''
            if length > 0:
                payload_data = self.socket.recv(length)
            
            return bytes([response]) + payload_data
            
        except Exception as e:
            print(f"✗ Command failed: {e}")
            return None
    
    def move_linear(self, positions: List[float], feed_rate: float = 1000.0) -> bool:
        """Linear move to positions with feed rate"""
        payload = struct.pack('<6ff', *positions, feed_rate)
        response = self._send_command(CMD_MOVE_LINEAR, payload)
        
        if response and response[0] == RESP_OK:
            print(f"✓ Linear move started: {positions}")
            return True
        return False
    
    def move_rapid(self, positions: List[float]) -> bool:
        """Rapid move to positions"""
        payload = struct.pack('<6f', *positions)
        response = self._send_command(CMD_MOVE_RAPID, payload)
        
        if response and response[0] == RESP_OK:
            print(f"✓ Rapid move started: {positions}")
            return True
        return False
    
    def move_joint(self, positions: List[float], 
                   velocities: Optional[List[float]] = None,
                   accelerations: Optional[List[float]] = None) -> bool:
        """Joint move with individual axis control"""
        if velocities is None:
            velocities = [1000.0] * 6
        if accelerations is None:
            accelerations = [500.0] * 6
        
        payload = struct.pack('<18f', *positions, *velocities, *accelerations)
        response = self._send_command(CMD_MOVE_JOINT, payload)
        
        if response and response[0] == RESP_OK:
            print(f"✓ Joint move started")
            return True
        return False
    
    def home(self) -> bool:
        """Home all axes"""
        response = self._send_command(CMD_HOME)
        
        if response and response[0] == RESP_OK:
            print("✓ Homing started")
            return True
        return False
    
    def stop(self) -> bool:
        """Emergency stop"""
        response = self._send_command(CMD_STOP)
        
        if response and response[0] == RESP_OK:
            print("✓ Emergency stop")
            return True
        return False
    
    def get_position(self) -> Optional[List[float]]:
        """Get current positions"""
        response = self._send_command(CMD_GET_POSITION)
        
        if response and response[0] == RESP_POSITION:
            positions = struct.unpack('<6f', response[1:25])
            return list(positions)
        return None
    
    def set_position(self, positions: List[float]) -> bool:
        """Set current position (zero)"""
        payload = struct.pack('<6f', *positions)
        response = self._send_command(CMD_SET_POSITION, payload)
        
        if response and response[0] == RESP_OK:
            print(f"✓ Position set: {positions}")
            return True
        return False
    
    def get_status(self) -> Optional[dict]:
        """Get system status"""
        response = self._send_command(CMD_GET_STATUS)
        
        if response and response[0] == RESP_STATUS:
            data = struct.unpack('<BBHHI', response[1:11])
            return {
                'is_moving': bool(data[0]),
                'motors_enabled': bool(data[1]),
                'buffer_free': data[2],
                'buffer_used': data[3],
                'error_flags': data[4]
            }
        return None
    
    def set_config(self, steps_per_mm: List[float],
                   max_velocity: List[float],
                   max_acceleration: List[float]) -> bool:
        """Set configuration"""
        payload = struct.pack('<18f', *steps_per_mm, *max_velocity, *max_acceleration)
        response = self._send_command(CMD_SET_CONFIG, payload)
        
        if response and response[0] == RESP_OK:
            print("✓ Configuration updated")
            return True
        return False
    
    def enable(self) -> bool:
        """Enable motors"""
        response = self._send_command(CMD_ENABLE)
        
        if response and response[0] == RESP_OK:
            print("✓ Motors enabled")
            return True
        return False
    
    def disable(self) -> bool:
        """Disable motors"""
        response = self._send_command(CMD_DISABLE)
        
        if response and response[0] == RESP_OK:
            print("✓ Motors disabled")
            return True
        return False
    
    def clear_buffer(self) -> bool:
        """Clear motion buffer"""
        response = self._send_command(CMD_CLEAR_BUFFER)
        
        if response and response[0] == RESP_OK:
            print("✓ Buffer cleared")
            return True
        return False
    
    def jtc_stream(self, timestamp_ms: int, positions: List[float],
                   velocities: List[float], accelerations: List[float],
                   effort: Optional[List[float]] = None) -> bool:
        """Stream JTC command (ROS2 Joint Trajectory Controller)"""
        if effort is None:
            effort = [0.0] * 6
        
        payload = struct.pack('<I24f', timestamp_ms, 
                            *positions, *velocities, *accelerations, *effort)
        response = self._send_command(CMD_JTC_STREAM, payload)
        
        return response and response[0] == RESP_OK
    
    def set_direct(self, positions: List[float], velocities: List[float]) -> bool:
        """Set positions and velocities directly (no motion planning)"""
        payload = struct.pack('<12f', *positions, *velocities)
        response = self._send_command(CMD_SET_DIRECT, payload)
        
        if response and response[0] == RESP_OK:
            return True
        return False


def test_basic_motion():
    """Test basic motion commands"""
    client = StepperClient()
    
    if not client.connect():
        return
    
    try:
        print("\n=== Basic Motion Test ===")
        
        # Enable motors
        client.enable()
        time.sleep(0.5)
        
        # Get initial position
        print("\n1. Initial position:")
        pos = client.get_position()
        if pos:
            for i, p in enumerate(pos):
                print(f"   Axis {i}: {p:.2f} mm")
        
        # Get status
        print("\n2. Status:")
        status = client.get_status()
        if status:
            print(f"   Moving: {status['is_moving']}")
            print(f"   Enabled: {status['motors_enabled']}")
            print(f"   Buffer: {status['buffer_used']}/{status['buffer_free']}")
        
        # Linear move
        print("\n3. Linear move test...")
        target = [10.0, 5.0, 0.0, 0.0, 0.0, 0.0]
        client.move_linear(target, feed_rate=500.0)
        time.sleep(3.0)
        
        # Check position
        print("\n4. Position after move:")
        pos = client.get_position()
        if pos:
            for i, p in enumerate(pos):
                print(f"   Axis {i}: {p:.2f} mm")
        
        # Move back
        print("\n5. Moving back to origin...")
        client.move_linear([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], feed_rate=1000.0)
        time.sleep(2.0)
        
        print("\n✓ Test completed!")
        
    except KeyboardInterrupt:
        print("\n✗ Test interrupted!")
        client.stop()
    finally:
        client.disconnect()


def test_jtc_streaming():
    """Test JTC streaming (ROS2 compatible)"""
    client = StepperClient()
    
    if not client.connect():
        return
    
    try:
        print("\n=== JTC Streaming Test (ROS2 Compatible) ===")
        
        client.enable()
        time.sleep(0.5)
        
        # Stream trajectory at 100Hz
        rate = 100  # Hz
        duration = 5.0  # seconds
        samples = int(rate * duration)
        
        print(f"Streaming {samples} samples at {rate}Hz...")
        
        start_time = time.time()
        for i in range(samples):
            t = i / rate
            
            # Simple sinusoidal trajectory
            positions = [
                10.0 * np.sin(2 * np.pi * 0.5 * t),
                5.0 * np.cos(2 * np.pi * 0.5 * t),
                0.0, 0.0, 0.0, 0.0
            ]
            
            velocities = [
                10.0 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t),
                -5.0 * 2 * np.pi * 0.5 * np.sin(2 * np.pi * 0.5 * t),
                0.0, 0.0, 0.0, 0.0
            ]
            
            accelerations = [0.0] * 6
            
            timestamp_ms = int(t * 1000)
            client.jtc_stream(timestamp_ms, positions, velocities, accelerations)
            
            # Maintain rate
            elapsed = time.time() - start_time
            target_time = (i + 1) / rate
            sleep_time = target_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print("✓ Streaming completed!")
        
    except KeyboardInterrupt:
        print("\n✗ Streaming interrupted!")
        client.stop()
    finally:
        client.disconnect()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "jtc":
        test_jtc_streaming()
    else:
        test_basic_motion()
