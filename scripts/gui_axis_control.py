#!/usr/bin/env python3
"""
Simple GUI for single axis debugging
Move individual axes left/right for testing
"""

import tkinter as tk
from tkinter import ttk
import socket
import struct
import threading
import math

STEPPER_PROTOCOL_VERSION = 1
CMD_MOVE_SINGLE = 0x03
CMD_GET_STATE = 0x10
RESP_OK = 0x00
RESP_STATE = 0x02

# Motor configuration
STEPS_PER_REV = 200
MICROSTEPS = 16
GEAR_RATIOS = [13.5, 150.0, 150.0, 48.0, 27.3375, 10.0]  # X, Y, Z, A, B, C


class AxisControlGUI:
    def __init__(self, host="192.168.178.159", port=8888):
        self.host = host
        self.port = port
        self.socket = None
        self.sequence = 0
        self.connected = False
        
        self.root = tk.Tk()
        self.root.title("Stepper Axis Control")
        self.root.geometry("600x500")
        
        self.axis_names = ['X', 'Y', 'Z', 'A', 'B', 'C']
        self.positions = [0.0] * 6  # In degrees
        
        # Calculate steps per revolution for each axis
        self.steps_per_rev = [STEPS_PER_REV * MICROSTEPS * gr for gr in GEAR_RATIOS]
        
        self.create_widgets()
        
    def create_widgets(self):
        # Connection frame
        conn_frame = ttk.Frame(self.root, padding="10")
        conn_frame.grid(row=0, column=0, sticky=(tk.W, tk.E))
        
        ttk.Label(conn_frame, text="Host:").grid(row=0, column=0)
        self.host_entry = ttk.Entry(conn_frame, width=20)
        self.host_entry.insert(0, self.host)
        self.host_entry.grid(row=0, column=1, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=3, padx=10)
        
        # Axis controls
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Label(control_frame, text="Axis", font=('Arial', 10, 'bold')).grid(row=0, column=0, padx=5)
        ttk.Label(control_frame, text="Position", font=('Arial', 10, 'bold')).grid(row=0, column=1, padx=5)
        ttk.Label(control_frame, text="Step Size (°)", font=('Arial', 10, 'bold')).grid(row=0, column=2, padx=5)
        ttk.Label(control_frame, text="Control", font=('Arial', 10, 'bold')).grid(row=0, column=3, columnspan=2, padx=5)
        
        self.position_labels = []
        self.step_entries = []
        
        for i, name in enumerate(self.axis_names):
            row = i + 1
            
            # Axis name
            ttk.Label(control_frame, text=f"{name} (Axis {i})", font=('Arial', 10)).grid(row=row, column=0, padx=5, pady=5)
            
            # Position display
            pos_label = ttk.Label(control_frame, text="0.00°", font=('Arial', 10))
            pos_label.grid(row=row, column=1, padx=5)
            self.position_labels.append(pos_label)
            
            # Step size entry
            step_entry = ttk.Entry(control_frame, width=10)
            step_entry.insert(0, "5.0")
            step_entry.grid(row=row, column=2, padx=5)
            self.step_entries.append(step_entry)
            
            # Left button
            left_btn = ttk.Button(control_frame, text="◄ Left", 
                                 command=lambda ax=i: self.move_axis(ax, -1))
            left_btn.grid(row=row, column=3, padx=2)
            
            # Right button
            right_btn = ttk.Button(control_frame, text="Right ►", 
                                  command=lambda ax=i: self.move_axis(ax, 1))
            right_btn.grid(row=row, column=4, padx=2)
        
        # Zero all button
        zero_frame = ttk.Frame(self.root, padding="10")
        zero_frame.grid(row=2, column=0)
        
        ttk.Button(zero_frame, text="Zero All Axes", command=self.zero_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(zero_frame, text="Refresh Positions", command=self.update_positions).pack(side=tk.LEFT, padx=5)
        
    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        try:
            self.host = self.host_entry.get()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2.0)
            self.socket.connect((self.host, self.port))
            self.connected = True
            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.update_positions()
        except Exception as e:
            self.status_label.config(text=f"Error: {e}", foreground="red")
    
    def disconnect(self):
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
    
    def _send_command(self, command: int, payload: bytes = b''):
        if not self.connected:
            return None
        
        self.sequence += 1
        header = struct.pack('<BBHI', STEPPER_PROTOCOL_VERSION, command, len(payload), self.sequence)
        
        try:
            self.socket.sendall(header + payload)
            resp_header = self.socket.recv(8)
            if len(resp_header) < 8:
                return None
            
            version, response, length, seq = struct.unpack('<BBHI', resp_header)
            payload_data = b''
            if length > 0:
                payload_data = self.socket.recv(length)
            
            return bytes([response]) + payload_data
        except:
            return None
    
    def deg_to_steps(self, degrees, axis):
        """Convert degrees to steps for a specific axis"""
        radians = math.radians(degrees)
        steps = radians * (self.steps_per_rev[axis] / (2.0 * math.pi))
        return steps
    
    def steps_to_deg(self, steps, axis):
        """Convert steps to degrees for a specific axis"""
        radians = steps * (2.0 * math.pi / self.steps_per_rev[axis])
        degrees = math.degrees(radians)
        return degrees
    
    def move_axis(self, axis: int, direction: int):
        if not self.connected:
            return
        
        try:
            step_size_deg = float(self.step_entries[axis].get())
            target_position_deg = self.positions[axis] + (step_size_deg * direction)
            
            # Convert degrees to steps for STM32
            target_position_steps = self.deg_to_steps(target_position_deg, axis)
            velocity_steps = self.deg_to_steps(20.0, axis)  # 20°/s
            
            payload = struct.pack('<Bfff', axis, target_position_steps, velocity_steps, 50.0)
            response = self._send_command(CMD_MOVE_SINGLE, payload)
            
            if response and response[0] == RESP_OK:
                self.positions[axis] = target_position_deg
                self.position_labels[axis].config(text=f"{target_position_deg:.2f}°")
        except ValueError:
            pass
    
    def zero_all(self):
        if not self.connected:
            return
        
        for axis in range(6):
            velocity_steps = self.deg_to_steps(20.0, axis)
            payload = struct.pack('<Bfff', axis, 0.0, velocity_steps, 50.0)
            self._send_command(CMD_MOVE_SINGLE, payload)
            self.positions[axis] = 0.0
            self.position_labels[axis].config(text="0.00°")
    
    def update_positions(self):
        if not self.connected:
            return
        
        response = self._send_command(CMD_GET_STATE)
        if response and response[0] == RESP_STATE:
            positions_steps = struct.unpack('<6f', response[1:25])
            for i, pos_steps in enumerate(positions_steps):
                pos_deg = self.steps_to_deg(pos_steps, i)
                self.positions[i] = pos_deg
                self.position_labels[i].config(text=f"{pos_deg:.2f}°")
    
    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    gui = AxisControlGUI()
    gui.run()
