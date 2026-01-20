#!/usr/bin/env python3
"""
Test Direct Position/Velocity Control
Simple streaming without motion planning
"""

import socket
import struct
import time
import numpy as np
from typing import List

# Protocol constants
STEPPER_PROTOCOL_VERSION = 1
CMD_SET_DIRECT = 0x0E
CMD_GET_POSITION = 0x06
CMD_GET_STATUS = 0x08
CMD_ENABLE = 0x0A
RESP_OK = 0x00
RESP_POSITION = 0x02
RESP_STATUS = 0x03

class DirectControlClient:
    def __init__(self, host: str = "192.168.178.159", port: int = 8888):
        self.host = host
        self.port = port
        self.socket = None
        self.sequence = 0
        
    def connect(self) -> bool:
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2.0)
            self.socket.connect((self.host, self.port))
            print(f"✓ Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def disconnect(self):
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def _send_command(self, command: int, payload: bytes = b''):
        self.sequence += 1
        header = struct.pack('<BBHI', STEPPER_PROTOCOL_VERSION, command, len(payload), self.sequence)
        packet = header + payload
        self.socket.sendall(packet)
        
        # Receive response
        resp_header = self.socket.recv(8)
        if len(resp_header) < 8:
            return None
        
        version, response, length, seq = struct.unpack('<BBHI', resp_header)
        payload_data = b''
        if length > 0:
            payload_data = self.socket.recv(length)
        
        return bytes([response]) + payload_data
    
    def set_direct(self, positions: List[float], velocities: List[float]) -> bool:
        """Set positions and velocities directly"""
        payload = struct.pack('<12f', *positions, *velocities)
        response = self._send_command(CMD_SET_DIRECT, payload)
        return response and response[0] == RESP_OK
    
    def get_position(self):
        """Get current positions"""
        response = self._send_command(CMD_GET_POSITION)
        if response and response[0] == RESP_POSITION:
            positions = struct.unpack('<6f', response[1:25])
            return list(positions)
        return None
    
    def get_status(self):
        """Get system status"""
        response = self._send_command(CMD_GET_STATUS)
        if response and response[0] == RESP_STATUS:
            data = struct.unpack('<BBHHI', response[1:11])
            return {
                'is_moving': bool(data[0]),
                'motors_enabled': bool(data[1]),
                'buffer_free': data[2],
                'buffer_used': data[3]
            }
        return None
    
    def enable(self) -> bool:
        """Enable motors"""
        response = self._send_command(CMD_ENABLE)
        return response and response[0] == RESP_OK


def test_direct_streaming():
    """Test direct position/velocity streaming"""
    client = DirectControlClient()
    
    if not client.connect():
        return
    
    try:
        print("\n=== Direct Position/Velocity Control Test ===\n")
        
        # Enable motors
        if client.enable():
            print("✓ Motors enabled")
        time.sleep(0.5)
        
        # Get initial position
        pos = client.get_position()
        if pos:
            print(f"Initial position: {[f'{p:.2f}' for p in pos]}")
        
        # Test 1: Simple position streaming
        print("\n1. Simple position streaming (10Hz)...")
        rate = 10  # Hz
        duration = 3.0
        samples = int(rate * duration)
        
        for i in range(samples):
            t = i / rate
            
            # Simple sinusoidal trajectory
            positions = [
                5.0 * np.sin(2 * np.pi * 0.5 * t),
                2.5 * np.cos(2 * np.pi * 0.5 * t),
                0.0, 0.0, 0.0, 0.0
            ]
            
            velocities = [
                5.0 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t),
                -2.5 * 2 * np.pi * 0.5 * np.sin(2 * np.pi * 0.5 * t),
                0.0, 0.0, 0.0, 0.0
            ]
            
            if client.set_direct(positions, velocities):
                if i % 10 == 0:
                    print(f"   t={t:.1f}s: pos=[{positions[0]:.2f}, {positions[1]:.2f}]")
            else:
                print(f"   ✗ Failed at t={t:.1f}s")
            
            time.sleep(1.0 / rate)
        
        print("✓ Streaming completed")
        
        # Get final position
        time.sleep(0.5)
        pos = client.get_position()
        if pos:
            print(f"\nFinal position: {[f'{p:.2f}' for p in pos]}")
        
        # Test 2: Return to zero
        print("\n2. Returning to zero...")
        for i in range(20):
            client.set_direct([0.0]*6, [10.0]*6)
            time.sleep(0.1)
        
        pos = client.get_position()
        if pos:
            print(f"Position at zero: {[f'{p:.2f}' for p in pos]}")
        
        print("\n✓ Test completed!")
        
    except KeyboardInterrupt:
        print("\n✗ Test interrupted!")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        client.disconnect()


def test_high_frequency():
    """Test high-frequency streaming (100Hz)"""
    client = DirectControlClient()
    
    if not client.connect():
        return
    
    try:
        print("\n=== High-Frequency Streaming Test (100Hz) ===\n")
        
        client.enable()
        time.sleep(0.5)
        
        rate = 100  # Hz
        duration = 5.0
        samples = int(rate * duration)
        
        print(f"Streaming {samples} samples at {rate}Hz...")
        
        start_time = time.time()
        success_count = 0
        
        for i in range(samples):
            t = i / rate
            
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
            
            if client.set_direct(positions, velocities):
                success_count += 1
            
            # Maintain rate
            elapsed = time.time() - start_time
            target_time = (i + 1) / rate
            sleep_time = target_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        elapsed_total = time.time() - start_time
        actual_rate = samples / elapsed_total
        
        print(f"\n✓ Streaming completed!")
        print(f"   Success rate: {success_count}/{samples} ({100*success_count/samples:.1f}%)")
        print(f"   Actual rate: {actual_rate:.1f}Hz")
        print(f"   Duration: {elapsed_total:.2f}s")
        
    except KeyboardInterrupt:
        print("\n✗ Test interrupted!")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        client.disconnect()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "fast":
        test_high_frequency()
    else:
        test_direct_streaming()
