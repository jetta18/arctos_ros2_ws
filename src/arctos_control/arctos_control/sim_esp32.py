import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class SimEsp32Node(Node):
    def __init__(self):
        super().__init__('sim_esp32')
        # Parameters
        self.declare_parameter('joint_commands_topic', '/topic_based_joint_commands')
        self.declare_parameter('joint_states_topic', '/topic_based_joint_states')
        self.declare_parameter('joint_names', ['X_joint', 'Y_joint', 'Z_joint', 'A_joint', 'B_joint', 'C_joint'])
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_velocity', 1.0)  # rad/s simulated tracking speed

        self.cmd_topic = self.get_parameter('joint_commands_topic').get_parameter_value().string_value
        self.state_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.joint_names: List[str] = [s for s in self.get_parameter('joint_names').get_parameter_value().string_array_value]
        self.rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.max_vel = self.get_parameter('max_velocity').get_parameter_value().double_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Command subscriber (what the PC sends to the robot)
        self.cmd_sub = self.create_subscription(JointState, self.cmd_topic, self.on_command, qos)
        # State publisher (what the robot reports back)
        self.state_pub = self.create_publisher(JointState, self.state_topic, qos)

        # Internal state
        self.last_cmd = JointState()
        self.last_cmd.name = list(self.joint_names)
        self.last_cmd.position = [0.0] * len(self.joint_names)
        self.last_cmd.velocity = [0.0] * len(self.joint_names)

        self.state = JointState()
        self.state.name = list(self.joint_names)
        self.state.position = [0.0] * len(self.joint_names)
        self.state.velocity = [0.0] * len(self.joint_names)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(1e-3, self.rate_hz), self.on_timer)
        self.get_logger().info(
            f"Sim ESP32 ready. Subscribing to '{self.cmd_topic}', publishing states to '{self.state_topic}' ({self.rate_hz} Hz)."
        )

    def on_command(self, msg: JointState):
        # Align incoming ordering to our known joint_names if possible
        idx_map = {name: i for i, name in enumerate(msg.name)} if msg.name else {}
        for i, jn in enumerate(self.joint_names):
            if idx_map:
                j = idx_map.get(jn, None)
                if j is not None and j < len(msg.position):
                    self.last_cmd.position[i] = float(msg.position[j])
                if j is not None and msg.velocity and j < len(msg.velocity):
                    self.last_cmd.velocity[i] = float(msg.velocity[j])
            else:
                # If no names sent, assume arrays match
                if i < len(msg.position):
                    self.last_cmd.position[i] = float(msg.position[i])
                if msg.velocity and i < len(msg.velocity):
                    self.last_cmd.velocity[i] = float(msg.velocity[i])
        self.last_time = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        # Simple first-order tracking towards commanded position using max velocity limit
        new_pos = []
        new_vel = []
        for i in range(len(self.joint_names)):
            err = self.last_cmd.position[i] - self.state.position[i]
            # bound step by max_vel
            step = max(-self.max_vel * dt, min(self.max_vel * dt, err))
            v = 0.0 if dt <= 0.0 else step / dt
            new_pos.append(self.state.position[i] + step)
            new_vel.append(v)
        self.state.header.stamp = now.to_msg()
        self.state.position = new_pos
        self.state.velocity = new_vel
        self.state_pub.publish(self.state)


def main():
    rclpy.init()
    node = SimEsp32Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
