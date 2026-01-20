#!/usr/bin/env python3
"""
Clean Stepper Motor Control Client
Minimal protocol for ROS2 integration and GUI debugging
"""

import socket
import struct
import time
from typing import List, Optional

STEPPER_PROTOCOL_VERSION = 1

CMD_JTC_STREAM = 0x01
CMD_SET_DIRECT = 0x02
CMD_MOVE_SINGLE = 0x03
CMD_GET_STATE = 0x10

RESP_OK = 0x00
RESP_ERROR = 0x01
RESP_STATE = 0x02


class StepperController:
    """Clean stepper motor controller client"""
    
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
            print(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def _send_command(self, command: int, payload: bytes = b'') -> Optional[bytes]:
        self.sequence += 1
        header = struct.pack('<BBHI', STEPPER_PROTOCOL_VERSION, command, len(payload), self.sequence)
        packet = header + payload
        
        try:
            self.socket.sendall(packet)
            resp_header = self.socket.recv(8)
            if len(resp_header) < 8:
                return None
            
            version, response, length, seq = struct.unpack('<BBHI', resp_header)
            payload_data = b''
            if length > 0:
                payload_data = self.socket.recv(length)
            
            return bytes([response]) + payload_data
        except Exception as e:
            print(f"Command failed: {e}")
            return None
    
    def jtc_stream(self, timestamp_ms: int, positions: List[float], 
                   velocities: List[float], accelerations: List[float]) -> bool:
        payload = struct.pack('<I18f', timestamp_ms, *positions, *velocities, *accelerations)
        response = self._send_command(CMD_JTC_STREAM, payload)
        return response and response[0] == RESP_OK
    
    def set_direct(self, positions: List[float], velocities: List[float]) -> bool:
        payload = struct.pack('<12f', *positions, *velocities)
        response = self._send_command(CMD_SET_DIRECT, payload)
        return response and response[0] == RESP_OK
    
    def move_single_axis(self, axis: int, position: float, 
                         velocity: float = 10.0, acceleration: float = 50.0) -> bool:
        payload = struct.pack('<Bfff', axis, position, velocity, acceleration)
        response = self._send_command(CMD_MOVE_SINGLE, payload)
        return response and response[0] == RESP_OK
    
    def get_state(self) -> Optional[dict]:
        response = self._send_command(CMD_GET_STATE)
        
        if response and response[0] == RESP_STATE:
            positions = struct.unpack('<6f', response[1:25])
            velocities = struct.unpack('<6f', response[25:49])
            return {
                'positions': list(positions),
                'velocities': list(velocities)
            }
        return None


if __name__ == "__main__":
    import sys
    
    controller = StepperController()
    if not controller.connect():
        sys.exit(1)
    
    try:
        print("\nTesting single axis control...")
        
        for axis in range(6):
            print(f"\nAxis {axis}:")
            controller.move_single_axis(axis, 10.0, velocity=20.0)
            time.sleep(1.0)
            
            state = controller.get_state()
            if state:
                print(f"  Position: {state['positions'][axis]:.2f}")
            
            controller.move_single_axis(axis, 0.0, velocity=20.0)
            time.sleep(1.0)
        
        print("\nTest completed!")
        
    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        controller.disconnect()
