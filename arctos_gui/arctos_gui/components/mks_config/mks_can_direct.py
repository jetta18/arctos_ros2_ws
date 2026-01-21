"""Direct CAN client for MKS servo motors using mks-servo-can library.

Replaces the ROS2 service-based client with direct CAN communication.
"""

import can
import threading
import logging
from typing import Optional, Dict, Any, List
from mks_servo_can import MksServo
from mks_servo_can.mks_enums import (
    Enable, SuccessStatus, WorkMode, Direction, 
    CalibrationResult, GoHomeResult, MotorStatus
)


class MKSCanDirect:
    """Direct CAN interface for MKS servo motors.
    
    Manages CAN bus connection and provides methods to configure/control
    multiple MKS servo motors directly via CAN.
    """
    
    def __init__(self, interface: str = 'socketcan', channel: str = 'can0', bitrate: int = 500000):
        """Initialize CAN interface.
        
        Args:
            interface: CAN interface type ('socketcan' for Linux)
            channel: CAN channel name ('can0')
            bitrate: CAN bitrate (500000 for MKS servos)
        """
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None
        self.notifier: Optional[can.Notifier] = None
        self.servos: Dict[int, MksServo] = {}
        self._lock = threading.Lock()
        self._connected = False
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def connect(self) -> bool:
        """Connect to CAN interface.
        
        Returns:
            True if connection successful, False otherwise
        """
        with self._lock:
            if self._connected:
                return True
                
            try:
                # Create CAN bus
                self.bus = can.interface.Bus(
                    interface=self.interface,
                    channel=self.channel,
                    bitrate=self.bitrate
                )
                
                # Create notifier for async message handling
                self.notifier = can.Notifier(self.bus, [])
                
                self._connected = True
                self.logger.info(f"Connected to CAN {self.channel} at {self.bitrate} baud")
                return True
                
            except Exception as e:
                self.logger.error(f"Failed to connect to CAN: {e}")
                self._cleanup()
                return False
    
    def disconnect(self) -> None:
        """Disconnect from CAN interface."""
        with self._lock:
            self._cleanup()
            self.logger.info("Disconnected from CAN")
    
    def _cleanup(self) -> None:
        """Clean up CAN resources."""
        if self.notifier:
            self.notifier.stop()
            self.notifier = None
        if self.bus:
            self.bus.shutdown()
            self.bus = None
        self.servos.clear()
        self._connected = False
    
    def is_connected(self) -> bool:
        """Check if CAN interface is connected.
        
        Returns:
            True if connected, False otherwise
        """
        return self._connected
    
    def _get_servo(self, motor_id: int) -> Optional[MksServo]:
        """Get or create servo instance for motor ID.
        
        Args:
            motor_id: Motor ID (1-6)
            
        Returns:
            MksServo instance or None if not connected
        """
        if not self._connected:
            return None
            
        if motor_id not in self.servos:
            self.servos[motor_id] = MksServo(self.bus, self.notifier, motor_id)
            
        return self.servos[motor_id]
    
    # Motor Configuration Methods
    
    def calibrate_motor(self, motor_id: int) -> Dict[str, Any]:
        """Calibrate motor encoder.
        
        Args:
            motor_id: Motor ID to calibrate
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            result = servo.nb_calibrate_encoder()
            if result == CalibrationResult.CalibratedSuccess:
                return {"success": True, "message": "Calibration successful"}
            else:
                return {"success": False, "message": f"Calibration failed: {result}"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def set_work_mode(self, motor_id: int, work_mode: int) -> Dict[str, Any]:
        """Set work mode for motor.
        
        Args:
            motor_id: Motor ID
            work_mode: Work mode value (0-5)
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            mode = WorkMode(work_mode)
            result = servo.set_work_mode(mode)
            return {"success": result == SuccessStatus.Success, 
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def set_working_current(self, motor_id: int, current_ma: int) -> Dict[str, Any]:
        """Set working current.
        
        Args:
            motor_id: Motor ID
            current_ma: Current in milliamperes
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            result = servo.set_working_current(current_ma)
            return {"success": result == SuccessStatus.Success,
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def set_holding_current(self, motor_id: int, percentage: int) -> Dict[str, Any]:
        """Set holding current percentage.
        
        Args:
            motor_id: Motor ID
            percentage: Holding current as percentage (0-100)
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            # Convert percentage to library enum (0-8 = 10%-90%)
            if percentage < 10:
                enum_val = 0
            elif percentage > 90:
                enum_val = 8
            else:
                enum_val = (percentage - 10) // 10
                
            result = servo.set_holding_current(enum_val)
            return {"success": result == SuccessStatus.Success,
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def set_subdivision(self, motor_id: int, subdivision: int) -> Dict[str, Any]:
        """Set microstep subdivision.
        
        Args:
            motor_id: Motor ID
            subdivision: Subdivision value (0-8 for 1-256 microsteps)
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            result = servo.set_subdivisions(subdivision)
            return {"success": result == SuccessStatus.Success,
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def restore_defaults(self, motor_id: int) -> Dict[str, Any]:
        """Restore factory defaults.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            result = servo.restore_default_parameters()
            return {"success": result == SuccessStatus.Success,
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    # Homing Methods
    
    def set_home_parameters(self, motor_id: int, trigger_level: int, direction: int,
                           speed_rpm: int, enable_limit: bool) -> Dict[str, Any]:
        """Set homing parameters.
        
        Args:
            motor_id: Motor ID
            trigger_level: Trigger level (0=Low, 1=High)
            direction: Direction (0=Clockwise, 1=Counter-Clockwise)
            speed_rpm: Homing speed in RPM
            enable_limit: Enable limit switch
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            # Map parameters to library enums
            trigger = 0 if trigger_level == 0 else 1
            dir_val = Direction.CW if direction == 0 else Direction.CCW
            
            result = servo.set_home(trigger, dir_val, speed_rpm, enable_limit)
            return {"success": result == SuccessStatus.Success,
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def go_home(self, motor_id: int) -> Dict[str, Any]:
        """Execute homing sequence.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            result = servo.nb_go_home()
            if result == GoHomeResult.Start:
                return {"success": True, "message": "Homing started"}
            else:
                return {"success": False, "message": f"Homing failed: {result}"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def set_zero_position(self, motor_id: int) -> Dict[str, Any]:
        """Set current position as zero.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            result = servo.set_current_axis_to_zero()
            return {"success": result == SuccessStatus.Success,
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def set_limit_remap(self, motor_id: int, enable: bool) -> Dict[str, Any]:
        """Enable/disable limit switch remapping.
        
        Args:
            motor_id: Motor ID
            enable: True to enable, False to disable
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            result = servo.set_limit_port_remap(enable)
            return {"success": result == SuccessStatus.Success,
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    # Motor Control Methods
    
    def enable_motor(self, motor_id: int, enable: bool) -> Dict[str, Any]:
        """Enable or disable motor.
        
        Args:
            motor_id: Motor ID
            enable: True to enable, False to disable
            
        Returns:
            Dict with success status and message
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            state = Enable.Enable if enable else Enable.Disable
            result = servo.enable_motor(state)
            return {"success": result == SuccessStatus.Success,
                   "message": "Success" if result == SuccessStatus.Success else "Failed"}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def query_status(self, motor_id: int) -> Dict[str, Any]:
        """Query motor status.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dict with success status and motor status info
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            status = servo.query_motor_status()
            return {"success": True, "message": "Status read", 
                   "motor_status": status}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    # Data Reading Methods
    
    def read_encoder(self, motor_id: int) -> Dict[str, Any]:
        """Read encoder position.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dict with success status and encoder data
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            value = servo.read_encoder_value_addition()
            # Convert to angle (assuming 1 unit = 0.01 degrees based on docs)
            angle_deg = value * 0.01
            angle_rad = angle_deg * 3.14159265359 / 180.0
            
            return {"success": True, "message": "Encoder read",
                   "encoder_raw_value": value,
                   "encoder_angle_degrees": angle_deg,
                   "encoder_angle_radians": angle_rad}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def read_speed(self, motor_id: int) -> Dict[str, Any]:
        """Read motor speed.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dict with success status and speed data
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            speed_rpm = servo.read_motor_speed()
            # Convert to rad/s
            speed_rad_per_sec = speed_rpm * 2 * 3.14159265359 / 60.0
            
            return {"success": True, "message": "Speed read",
                   "speed_rpm": speed_rpm,
                   "speed_rad_per_sec": speed_rad_per_sec}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def read_io_status(self, motor_id: int) -> Dict[str, Any]:
        """Read I/O port status.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dict with success status and I/O data
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            # IO status returns a byte where each bit represents a pin
            io_byte = servo.read_io_port_status()
            
            # Extract individual pin states (assuming standard mapping)
            io_in1 = bool(io_byte & 0x01)  # Pin 1 - Home/Left Limit
            io_in2 = bool(io_byte & 0x02)  # Pin 2 - Right Limit
            io_out1 = bool(io_byte & 0x04)  # Pin 3 - Stall Detection
            io_out2 = bool(io_byte & 0x08)  # Pin 4
            
            return {"success": True, "message": "IO status read",
                   "io_in1": io_in1, "io_in2": io_in2,
                   "io_out1": io_out1, "io_out2": io_out2,
                   "stall_detected": io_out1}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
    
    def read_motor_status(self, motor_id: int) -> Dict[str, Any]:
        """Read detailed motor status.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dict with success status and detailed motor info
        """
        servo = self._get_servo(motor_id)
        if not servo:
            return {"success": False, "message": "CAN not connected"}
            
        try:
            status = servo.query_motor_status()
            is_running = servo.is_motor_running()
            
            # Map enum to readable values
            status_map = {
                MotorStatus.Fail: "Fail",
                MotorStatus.MotorStop: "Stopped",
                MotorStatus.MotorSpeedUp: "Speeding Up",
                MotorStatus.MotorSpeedDown: "Speeding Down",
                MotorStatus.MotorFullSpeed: "Full Speed",
                MotorStatus.MotorHoming: "Homing",
                MotorStatus.MotorIsCalibrating: "Calibrating"
            }
            
            status_text = status_map.get(status, "Unknown")
            
            return {"success": True, "message": "Motor status read",
                   "motor_enabled": status != MotorStatus.Fail,
                   "motor_moving": is_running,
                   "motor_calibrated": True,  # Assume calibrated if not actively calibrating
                   "motor_error": status == MotorStatus.Fail,
                   "motor_status_code": status.value,
                   "motor_status_text": status_text}
        except Exception as e:
            return {"success": False, "message": f"Error: {str(e)}"}
