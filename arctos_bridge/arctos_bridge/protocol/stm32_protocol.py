"""STM32 protocol client for the Arctos bridge.

Handles command sending (port 8888) and state broadcast receiving (port 8889).
"""

import socket
import struct
import threading
import time
from typing import Optional, Dict, Any

PROTOCOL_VERSION = 2
NUM_AXES = 6

HEADER_FMT = "<BBHI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
RESP_HEADER_FMT = "<BBHI"
RESP_HEADER_SIZE = struct.calcsize(RESP_HEADER_FMT)

# Command codes
CMD_MOVE_AXIS = 0x01
CMD_MOVE_ALL = 0x02
CMD_STOP = 0x03
CMD_SET_POSITION = 0x04
CMD_GET_STATE = 0x10
CMD_GET_ENDSTOPS = 0x11
CMD_PING = 0x20
CMD_SET_SERVO = 0x30
CMD_TRAJ_BEGIN = 0x40
CMD_TRAJ_POINT = 0x41
CMD_TRAJ_EXECUTE = 0x42
CMD_TRAJ_CANCEL = 0x43
CMD_SUBSCRIBE = 0x50
CMD_UNSUBSCRIBE = 0x51
CMD_HOME_START = 0x60
CMD_HOME_STOP = 0x61
CMD_HOME_TO_POSITION = 0x62
CMD_SET_HOME_OFFSET = 0x63
CMD_GET_HOME_OFFSET = 0x64
CMD_JOG_AXIS = 0x65

# Response codes
RESP_OK = 0x00
RESP_ERROR = 0x01
RESP_STATE = 0x02
RESP_ENDSTOPS = 0x03

# Broadcast
BROADCAST_STATE = 0x80
BROADCAST_HEADER_FMT = "<BBH"
BROADCAST_HEADER_SIZE = struct.calcsize(BROADCAST_HEADER_FMT)

BROADCAST_PAYLOAD_FMT = (
    "<"
    "I"
    f"{NUM_AXES}f"
    f"{NUM_AXES}f"
    "B"
    "I"
    "H"
    "H"
    "H"
    "H"
    "12I"
    "H"
    "B"
    "B"
    "B"
    "B"
    "I"
    "H"
)
BROADCAST_PAYLOAD_SIZE = struct.calcsize(BROADCAST_PAYLOAD_FMT)

# System state names
STATE_NAMES = {
    0: "IDLE",
    1: "MOVING",
    2: "TRAJ_LOADING",
    3: "TRAJ_RUNNING",
    4: "TRAJ_COMPLETE",
    5: "ERROR",
    6: "STOPPING",
}

ENDSTOP_AXIS_NAMES = ["X", "Y", "Z", "A", "B", "C"]
ENDSTOP_DIR_NAMES = ["MIN", "MAX"]

HOMING_STATE_NAMES = {
    0: "IDLE",
    1: "SEEKING",
    2: "RETURNING",
    3: "COMPLETE",
    4: "ERROR",
}


class STM32CommandClient:
    """Sends commands to the STM32 on the command port (8888)."""

    def __init__(self, host: str, port: int, timeout_s: float = 2.0):
        self._host = host
        self._port = port
        self._timeout = timeout_s
        self._sequence = 0
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self._timeout)

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _next_seq(self) -> int:
        self._sequence += 1
        return self._sequence

    def _send_cmd(self, command: int, payload: bytes = b"") -> int:
        if self._sock is None:
            raise ConnectionError("Not connected")
        seq = self._next_seq()
        header = struct.pack(HEADER_FMT, PROTOCOL_VERSION, command,
                             len(payload), seq)
        self._sock.sendto(header + payload, (self._host, self._port))
        return seq

    def _recv_response(self) -> tuple:
        if self._sock is None:
            raise ConnectionError("Not connected")
        data, _ = self._sock.recvfrom(512)
        if len(data) < RESP_HEADER_SIZE:
            raise ValueError(f"Response too short: {len(data)} bytes")
        version, resp_type, length, seq = struct.unpack_from(
            RESP_HEADER_FMT, data, 0)
        payload = data[RESP_HEADER_SIZE:]
        return version, resp_type, length, seq, payload

    def _parse_error(self, payload: bytes) -> str:
        if len(payload) >= 32:
            error_code = payload[0]
            message = payload[1:32].split(b"\x00")[0].decode(
                "utf-8", errors="replace")
            return f"Error {error_code}: {message}"
        return "Unknown error"

    def ping(self) -> bool:
        with self._lock:
            seq = self._send_cmd(CMD_PING)
            _, resp, _, rseq, _ = self._recv_response()
            return resp == RESP_OK and rseq == seq

    def subscribe(self, listen_port: int, rate_hz: int) -> bool:
        with self._lock:
            payload = struct.pack("<HB", listen_port, rate_hz)
            self._send_cmd(CMD_SUBSCRIBE, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False
            return resp == RESP_OK

    def unsubscribe(self) -> bool:
        with self._lock:
            self._send_cmd(CMD_UNSUBSCRIBE)
            _, resp, _, _, _ = self._recv_response()
            return resp == RESP_OK

    def stop(self) -> tuple:
        with self._lock:
            self._send_cmd(CMD_STOP)
            _, resp, _, _, payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(payload)
            return resp == RESP_OK, "OK"

    def set_servo(self, pulse_us: int, duration_ms: int) -> tuple:
        with self._lock:
            payload = struct.pack("<HH", pulse_us, duration_ms)
            self._send_cmd(CMD_SET_SERVO, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def move_all(self, positions_steps: list, max_velocity: float,
                 acceleration: float) -> tuple:
        with self._lock:
            payload = struct.pack(
                f"<{NUM_AXES}fff", *positions_steps, max_velocity, acceleration)
            self._send_cmd(CMD_MOVE_ALL, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def home_start(self, axis: int, direction: int,
                   velocity_steps_s: float) -> tuple:
        with self._lock:
            payload = struct.pack("<BBf", axis, direction, velocity_steps_s)
            self._send_cmd(CMD_HOME_START, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def home_stop(self) -> tuple:
        with self._lock:
            self._send_cmd(CMD_HOME_STOP)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def home_to_position(self, axis: int,
                         velocity_steps_s: float,
                         offset_steps: int) -> tuple:
        """Home axis to endstop then move to offset position."""
        with self._lock:
            payload = struct.pack("<Bfi", axis, velocity_steps_s, offset_steps)
            self._send_cmd(CMD_HOME_TO_POSITION, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def set_home_offset(self, axis: int, direction: int, offset_steps: int) -> tuple:
        """Set home offset and direction for an axis.
        
        Args:
            axis: Axis index 0-5
            direction: Endstop direction (0=MIN, 1=MAX)
            offset_steps: Signed offset from endstop to home position
        
        Returns:
            (success, message) tuple
        """
        with self._lock:
            payload = struct.pack("<BBi", axis, direction, offset_steps)
            self._send_cmd(CMD_SET_HOME_OFFSET, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def get_home_offset(self, axis: int) -> tuple:
        with self._lock:
            payload = struct.pack("<B", axis)
            self._send_cmd(CMD_GET_HOME_OFFSET, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, 0, self._parse_error(resp_payload)
            if len(resp_payload) >= 5:
                _, offset = struct.unpack_from("<Bi", resp_payload, 0)
                return True, offset, "OK"
            return False, 0, "Invalid response"

    def trajectory_upload_and_execute(
        self,
        trajectory_id: int,
        num_points: int,
        points: list,
    ) -> tuple:
        """Atomically upload all points and execute a trajectory.

        Holds the command lock for the entire sequence so no other
        command (e.g. move_all) can interleave.

        Args:
            trajectory_id: Unique trajectory identifier.
            num_points: Number of points to upload.
            points: List of (point_index, time_ms, positions_steps, velocities_steps).

        Returns:
            (success, message) tuple.
        """
        with self._lock:
            payload = struct.pack("<IH", trajectory_id, num_points)
            self._send_cmd(CMD_TRAJ_BEGIN, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            if resp != RESP_OK:
                return False, "TRAJ_BEGIN unexpected response"

            for point_index, time_ms, positions_steps, velocities_steps in points:
                pt_payload = struct.pack(
                    f"<IHI{NUM_AXES}f{NUM_AXES}f",
                    trajectory_id, point_index, time_ms,
                    *positions_steps, *velocities_steps)
                self._send_cmd(CMD_TRAJ_POINT, pt_payload)
                time.sleep(0.0005)

            payload = struct.pack("<I", trajectory_id)
            self._send_cmd(CMD_TRAJ_EXECUTE, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def trajectory_cancel(self) -> tuple:
        with self._lock:
            self._send_cmd(CMD_TRAJ_CANCEL)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def jog_axis(
        self, axis: int, distance_steps: int,
        velocity_steps_s: float, accel_steps_s2: float,
    ) -> tuple:
        """Jog a single axis by a relative distance."""
        with self._lock:
            payload = struct.pack(
                "<Biff", axis, distance_steps,
                velocity_steps_s, accel_steps_s2,
            )
            self._send_cmd(CMD_JOG_AXIS, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"

    def set_axis_position(self, axis: int, position_steps: float) -> tuple:
        """Set the step counter for a single axis to an arbitrary value.

        Args:
            axis: Axis index 0-5
            position_steps: New position value in steps (can be 0 to reset)

        Returns:
            (success, message) tuple
        """
        with self._lock:
            payload = struct.pack("<Bf", axis, position_steps)
            self._send_cmd(CMD_SET_POSITION, payload)
            _, resp, _, _, resp_payload = self._recv_response()
            if resp == RESP_ERROR:
                return False, self._parse_error(resp_payload)
            return resp == RESP_OK, "OK"


def parse_broadcast(data: bytes) -> Optional[Dict[str, Any]]:
    """Parse a state_broadcast_t packet into a dict."""
    if len(data) < BROADCAST_HEADER_SIZE:
        return None

    version, msg_type, length = struct.unpack_from(
        BROADCAST_HEADER_FMT, data, 0)

    if version != PROTOCOL_VERSION or msg_type != BROADCAST_STATE:
        return None

    payload_data = data[BROADCAST_HEADER_SIZE:]
    if len(payload_data) < BROADCAST_PAYLOAD_SIZE:
        return None

    values = struct.unpack_from(BROADCAST_PAYLOAD_FMT, payload_data, 0)
    idx = 0

    timestamp_ms = values[idx]; idx += 1
    positions = list(values[idx:idx + NUM_AXES]); idx += NUM_AXES
    velocities = list(values[idx:idx + NUM_AXES]); idx += NUM_AXES
    system_state = values[idx]; idx += 1
    trajectory_id = values[idx]; idx += 1
    traj_points_loaded = values[idx]; idx += 1
    traj_current_segment = values[idx]; idx += 1
    traj_total_segments = values[idx]; idx += 1
    endstop_states = values[idx]; idx += 1
    endstop_trigger_counts = list(values[idx:idx + 12]); idx += 12
    servo_pulse_us = values[idx]; idx += 1
    homing_active = values[idx]; idx += 1
    homing_axis = values[idx]; idx += 1
    homing_state = values[idx]; idx += 1
    is_homed_bitmask = values[idx]; idx += 1
    uptime_ms = values[idx]; idx += 1
    broadcast_seq = values[idx]; idx += 1

    return {
        "timestamp_ms": timestamp_ms,
        "positions": positions,
        "velocities": velocities,
        "system_state": system_state,
        "trajectory_id": trajectory_id,
        "traj_points_loaded": traj_points_loaded,
        "traj_current_segment": traj_current_segment,
        "traj_total_segments": traj_total_segments,
        "endstop_states": endstop_states,
        "endstop_trigger_counts": endstop_trigger_counts,
        "servo_pulse_us": servo_pulse_us,
        "homing_active": homing_active,
        "homing_axis": homing_axis,
        "homing_state": homing_state,
        "is_homed_bitmask": is_homed_bitmask,
        "uptime_ms": uptime_ms,
        "broadcast_seq": broadcast_seq,
    }


def triggered_endstop_names(endstop_states: int) -> list:
    """Return list of triggered endstop name strings."""
    triggered = []
    for ax_idx in range(6):
        for dir_idx in range(2):
            bit = ax_idx * 2 + dir_idx
            if endstop_states & (1 << bit):
                name = f"{ENDSTOP_AXIS_NAMES[ax_idx]}_{ENDSTOP_DIR_NAMES[dir_idx]}"
                triggered.append(name)
    return triggered
