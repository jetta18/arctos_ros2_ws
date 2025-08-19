import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class CommandWatchdog(Node):
    def __init__(self):
        super().__init__('arctos_command_watchdog')
        self.declare_parameter('joint_commands_topic', '/topic_based_joint_commands')
        self.declare_parameter('joint_states_topic', '/topic_based_joint_states')
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('estop_topic', '/arctos/estop')

        self.cmd_topic = self.get_parameter('joint_commands_topic').get_parameter_value().string_value
        self.state_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value
        self.estop_topic = self.get_parameter('estop_topic').get_parameter_value().string_value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.last_cmd_time = self.get_clock().now()
        self.cmd_sub = self.create_subscription(JointState, self.cmd_topic, self.on_cmd, qos)
        self.estop_pub = self.create_publisher(Bool, self.estop_topic, qos)

        self.timer = self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f"Watchdog monitoring '{self.cmd_topic}' with timeout {self.timeout_sec:.3f}s, publishing estop on '{self.estop_topic}'."
        )

    def on_cmd(self, _msg: JointState):
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time) > Duration(seconds=self.timeout_sec):
            # Publish estop true once per breach
            self.get_logger().warn('Command timeout exceeded. Triggering E-STOP.')
            self.estop_pub.publish(Bool(data=True))
            # Reset timer to avoid spamming
            self.last_cmd_time = now


def main():
    rclpy.init()
    node = CommandWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
