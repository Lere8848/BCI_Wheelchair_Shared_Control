# no_input_monitor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Bool
from rclpy.duration import Duration

class NoInputMonitorNode(Node):
    def __init__(self):
        super().__init__('no_input_monitor_node')
        self.user_input_sub = self.create_subscription(Int8, '/user_cmd', self.user_input_callback, 10)
        self.danger_pub = self.create_publisher(Bool, '/danger_stop', 10)

        self.last_input_time = None
        self.timer = self.create_timer(0.1, self.check_idle)
        self.danger_state = False  # 当前是否已处于 danger 状态

        self.get_logger().info('NoInputMonitorNode started. Monitoring user inactivity.')

    def user_input_callback(self, msg):
        self.last_input_time = self.get_clock().now()
        if self.danger_state:
            self.get_logger().info('User input detected, clearing danger_stop.')
            self.danger_pub.publish(Bool(data=False))
            self.danger_state = False

    def check_idle(self):
        now = self.get_clock().now()
        if self.last_input_time is None:
            return

        idle_time = now - self.last_input_time
        if idle_time > Duration(seconds=3.0) and not self.danger_state:
            self.get_logger().warn('No input for 3s. Triggering danger_stop.')
            self.danger_pub.publish(Bool(data=True))
            self.danger_state = True

def main(args=None):
    rclpy.init(args=args)
    node = NoInputMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
