import rclpy
from rclpy.node import Node
from arctos_motor_driver.srv import MKSMotorConfig, MKSMotorRead, MKSConnection


class MKSConfigClient(Node):
    
    def __init__(self):
        super().__init__('mks_config_client')
        
        self.config_client = self.create_client(MKSMotorConfig, 'mks_motor_config')
        self.read_client = self.create_client(MKSMotorRead, 'mks_motor_read')
        self.connection_client = self.create_client(MKSConnection, 'mks_connection')
        
        self.get_logger().info('MKS Config Client initialized')
    
    def wait_for_services(self, timeout_sec=5.0):
        config_ready = self.config_client.wait_for_service(timeout_sec=timeout_sec)
        read_ready = self.read_client.wait_for_service(timeout_sec=timeout_sec)
        connection_ready = self.connection_client.wait_for_service(timeout_sec=timeout_sec)
        return config_ready and read_ready and connection_ready
    
    def calibrate_motor(self, motor_id: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'calibrate'
        return self._send_config_request(request)
    
    def set_work_mode(self, motor_id: int, work_mode: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'set_work_mode'
        request.work_mode = work_mode
        return self._send_config_request(request)
    
    def set_working_current(self, motor_id: int, current_ma: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'set_current'
        request.working_current_ma = current_ma
        return self._send_config_request(request)
    
    def set_holding_current(self, motor_id: int, percentage: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'set_holding_current'
        request.holding_current_pct = percentage
        return self._send_config_request(request)
    
    def set_subdivision(self, motor_id: int, subdivision: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'set_subdivision'
        request.subdivision = subdivision
        return self._send_config_request(request)
    
    def restore_defaults(self, motor_id: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'restore_defaults'
        return self._send_config_request(request)
    
    def set_home_parameters(self, motor_id: int, trigger_level: int, direction: int, 
                           speed_rpm: int, enable_limit: bool):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'set_home_params'
        request.home_trigger_level = trigger_level
        request.home_direction = direction
        request.home_speed_rpm = speed_rpm
        request.home_enable_limit = enable_limit
        return self._send_config_request(request)
    
    def go_home(self, motor_id: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'go_home'
        return self._send_config_request(request)
    
    def set_zero_position(self, motor_id: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'set_zero'
        return self._send_config_request(request)
    
    def set_limit_remap(self, motor_id: int, enable: bool):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'set_limit_remap'
        request.limit_remap_enable = enable
        return self._send_config_request(request)
    
    def enable_motor(self, motor_id: int, enable: bool):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'enable'
        request.motor_enable = enable
        return self._send_config_request(request)
    
    def query_status(self, motor_id: int):
        request = MKSMotorConfig.Request()
        request.motor_id = motor_id
        request.command_type = 'query_status'
        return self._send_config_request(request)
    
    def read_encoder(self, motor_id: int):
        request = MKSMotorRead.Request()
        request.motor_id = motor_id
        request.read_type = 'encoder'
        return self._send_read_request(request)
    
    def read_speed(self, motor_id: int):
        request = MKSMotorRead.Request()
        request.motor_id = motor_id
        request.read_type = 'speed'
        return self._send_read_request(request)
    
    def read_io_status(self, motor_id: int):
        request = MKSMotorRead.Request()
        request.motor_id = motor_id
        request.read_type = 'io_status'
        return self._send_read_request(request)
    
    def read_motor_status(self, motor_id: int):
        request = MKSMotorRead.Request()
        request.motor_id = motor_id
        request.read_type = 'motor_status'
        return self._send_read_request(request)
    
    def _send_config_request(self, request):
        if not self.config_client.service_is_ready():
            self.get_logger().warning('Config service not ready')
            return None
        
        future = self.config_client.call_async(request)
        return future
    
    def _send_read_request(self, request):
        if not self.read_client.service_is_ready():
            self.get_logger().warning('Read service not ready')
            return None
        
        future = self.read_client.call_async(request)
        return future
    
    def connect_can(self):
        """Connect to CAN interface."""
        request = MKSConnection.Request()
        request.connect = True
        return self._send_connection_request(request)
    
    def disconnect_can(self):
        """Disconnect from CAN interface."""
        request = MKSConnection.Request()
        request.connect = False
        return self._send_connection_request(request)
    
    def _send_connection_request(self, request):
        if not self.connection_client.service_is_ready():
            self.get_logger().warning('Connection service not ready')
            return None
        
        future = self.connection_client.call_async(request)
        return future
