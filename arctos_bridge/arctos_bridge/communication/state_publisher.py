"""State message publishing logic for the Arctos bridge.

This module handles the conversion of raw STM32 state data into ROS2 messages
and publishes them to the appropriate topics.
"""

from typing import Dict, Any
from rclpy.node import Node

from arctos_msgs.msg import ArctosState, ArctosEndstops
from arctos_bridge.protocol.stm32_protocol import triggered_endstop_names
from arctos_bridge.protocol.unit_conversion import UnitConverter


class StatePublisher:
    """Converts STM32 state data to ROS2 messages and publishes them.
    
    Takes raw state dictionaries from the broadcast receiver, converts units,
    and publishes ArctosState and ArctosEndstops messages.
    """
    
    def __init__(
        self,
        node: Node,
        converter: UnitConverter,
        num_joints: int,
    ) -> None:
        """Initialize the state publisher.
        
        Args:
            node: ROS2 node for clock and logging
            converter: Unit converter for steps ↔ radians
            num_joints: Number of joints (typically 6)
        """
        self._node = node
        self._converter = converter
        self._num_joints = num_joints
    
    def create_state_message(self, state: Dict[str, Any]) -> ArctosState:
        """Create an ArctosState message from raw state data.
        
        Args:
            state: Raw state dictionary from parse_broadcast()
            
        Returns:
            Populated ArctosState message ready to publish
        """
        msg = ArctosState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        msg.positions = [
            self._converter.steps_to_rad(state["positions"][i], i)
            for i in range(self._num_joints)
        ]
        msg.velocities = [
            self._converter.steps_per_sec_to_rad_per_sec(state["velocities"][i], i)
            for i in range(self._num_joints)
        ]
        msg.positions_steps = [
            float(state["positions"][i]) for i in range(self._num_joints)
        ]
        
        msg.system_state = state["system_state"]
        msg.trajectory_id = state["trajectory_id"]
        msg.traj_points_loaded = state["traj_points_loaded"]
        msg.traj_current_segment = state["traj_current_segment"]
        msg.traj_total_segments = state["traj_total_segments"]
        msg.endstop_states = state["endstop_states"]
        msg.servo_pulse_us = state["servo_pulse_us"]
        msg.homing_active = bool(state["homing_active"])
        msg.homing_axis = state["homing_axis"]
        msg.homing_state = state["homing_state"]
        msg.is_homed_bitmask = state["is_homed_bitmask"]
        msg.stm32_uptime_ms = state["uptime_ms"]
        msg.broadcast_seq = state["broadcast_seq"]
        
        return msg
    
    def create_endstop_message(self, state: Dict[str, Any]) -> ArctosEndstops:
        """Create an ArctosEndstops message from raw state data.
        
        Args:
            state: Raw state dictionary from parse_broadcast()
            
        Returns:
            Populated ArctosEndstops message ready to publish
        """
        msg = ArctosEndstops()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.endstop_states = state["endstop_states"]
        msg.trigger_counts = [int(c) for c in state["endstop_trigger_counts"]]
        msg.triggered_names = triggered_endstop_names(state["endstop_states"])
        return msg
