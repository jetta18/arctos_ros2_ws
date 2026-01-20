#!/usr/bin/env python3
"""
Test all 6 axes individually
"""

import socket
import struct
import time
import numpy as np

STEPPER_PROTOCOL_VERSION = 1
CMD_SET_DIRECT = 0x0E
CMD_GET_POSITION = 0x06
CMD_ENABLE = 0x0A
RESP_OK = 0x00
RESP_POSITION = 0x02

class DirectControlClient:
    def __init__(self, host: str = "192.168.178.159", port: int = 8888):
        self.host = host
        self.port = port
        self.socket = None
        self.sequence = 0
        
    def connect(self):
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
    
    def _send_command(self, command: int, payload: bytes = b''):
        self.sequence += 1
        header = struct.pack('<BBHI', STEPPER_PROTOCOL_VERSION, command, len(payload), self.sequence)
        self.socket.sendall(header + payload)
        
        resp_header = self.socket.recv(8)
        if len(resp_header) < 8:
            return None
        
        version, response, length, seq = struct.unpack('<BBHI', resp_header)
        payload_data = b''
        if length > 0:
            payload_data = self.socket.recv(length)
        
        return bytes([response]) + payload_data
    
    def set_direct(self, positions, velocities):
        payload = struct.pack('<12f', *positions, *velocities)
        response = self._send_command(CMD_SET_DIRECT, payload)
        return response and response[0] == RESP_OK
    
    def get_position(self):
        response = self._send_command(CMD_GET_POSITION)
        if response and response[0] == RESP_POSITION:
            return list(struct.unpack('<6f', response[1:25]))
        return None
    
    def enable(self):
        response = self._send_command(CMD_ENABLE)
        return response and response[0] == RESP_OK


def test_all_axes():
    """Test each axis individually"""
    client = DirectControlClient()
    
    if not client.connect():
        return
    
    try:
        print("\n=== Testing All 6 Axes Individually ===\n")
        
        client.enable()
        time.sleep(0.5)
        
        axis_names = ['X', 'Y', 'Z', 'A', 'B', 'C']
        
        for axis_idx in range(6):
            print(f"\n--- Testing Axis {axis_idx + 1} ({axis_names[axis_idx]}) ---")
            
            # Move this axis only
            for step in range(20):
                t = step / 10.0
                
                positions = [0.0] * 6
                velocities = [0.0] * 6
                
                # Simple back-and-forth motion
                positions[axis_idx] = 5.0 * np.sin(2 * np.pi * 0.5 * t)
                velocities[axis_idx] = 5.0 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t)
                
                client.set_direct(positions, velocities)
                time.sleep(0.1)
            
            # Check position
            pos = client.get_position()
            if pos:
                print(f"   Final position: {axis_names[axis_idx]}={pos[axis_idx]:.2f} mm")
            
            # Return to zero
            print(f"   Returning to zero...")
            for _ in range(10):
                positions = [0.0] * 6
                velocities = [10.0] * 6
                client.set_direct(positions, velocities)
                time.sleep(0.1)
            
            time.sleep(0.5)
        
        print("\n✓ All axes tested!")
        
    except KeyboardInterrupt:
        print("\n✗ Test interrupted!")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        client.disconnect()


def test_simultaneous():
    """Test all axes moving simultaneously"""
    client = DirectControlClient()
    
    if not client.connect():
        return
    
    try:
        print("\n=== Testing All 6 Axes Simultaneously ===\n")
        
        client.enable()
        time.sleep(0.5)
        
        print("Moving all axes in circular pattern...")
        
        for step in range(30):
            t = step / 10.0
            
            # Each axis with different frequency/amplitude
            positions = [
                5.0 * np.sin(2 * np.pi * 0.5 * t),      # X
                5.0 * np.cos(2 * np.pi * 0.5 * t),      # Y
                3.0 * np.sin(2 * np.pi * 0.3 * t),      # Z
                2.0 * np.cos(2 * np.pi * 0.4 * t),      # A
                2.0 * np.sin(2 * np.pi * 0.6 * t),      # B
                1.0 * np.cos(2 * np.pi * 0.7 * t),      # C
            ]
            
            velocities = [
                5.0 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t),
                -5.0 * 2 * np.pi * 0.5 * np.sin(2 * np.pi * 0.5 * t),
                3.0 * 2 * np.pi * 0.3 * np.cos(2 * np.pi * 0.3 * t),
                -2.0 * 2 * np.pi * 0.4 * np.sin(2 * np.pi * 0.4 * t),
                2.0 * 2 * np.pi * 0.6 * np.cos(2 * np.pi * 0.6 * t),
                -1.0 * 2 * np.pi * 0.7 * np.sin(2 * np.pi * 0.7 * t),
            ]
            
            client.set_direct(positions, velocities)
            
            if step % 10 == 0:
                pos = client.get_position()
                if pos:
                    print(f"   t={t:.1f}s: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}, A={pos[3]:.1f}, B={pos[4]:.1f}, C={pos[5]:.1f}")
            
            time.sleep(0.1)
        
        print("\n✓ Simultaneous test completed!")
        
        # Return all to zero
        print("\nReturning all axes to zero...")
        for _ in range(20):
            client.set_direct([0.0]*6, [10.0]*6)
            time.sleep(0.1)
        
        pos = client.get_position()
        if pos:
            print(f"Final: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f}, A={pos[3]:.2f}, B={pos[4]:.2f}, C={pos[5]:.2f}")
        
    except KeyboardInterrupt:
        print("\n✗ Test interrupted!")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        client.disconnect()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "all":
        test_simultaneous()
    else:
        test_all_axes()
