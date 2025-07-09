# fusion_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int8MultiArray
from geometry_msgs.msg import Twist

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.user_cmd_sub = self.create_subscription(Int8, '/user_cmd', self.user_cmd_callback, 10)
        self.path_opt_sub = self.create_subscription(Int8MultiArray, '/path_options', self.path_callback, 10)

        self.path_options = [1, 1, 1]  # default all passable

    def path_callback(self, msg):
        if len(msg.data) == 3:
            self.path_options = msg.data

    def user_cmd_callback(self, msg):
        direction = msg.data  # 0=left, 1=forward, 2=right

        twist = Twist()

        if direction < 0 or direction > 2:
            self.get_logger().warn(f'Invalid direction: {direction}')
            return

        if self.path_options[direction] == 1:
            # Direction is safe, generate movement
            if direction == 0:  # left
                twist.angular.z = -0.4
            elif direction == 1:  # forward
                twist.linear.x = 0.3
            elif direction == 2:  # right
                twist.angular.z = 0.4
            self.get_logger().info(f'Executing user intent: {["LEFT","FORWARD","RIGHT"][direction]}')
        else:
            # Unsafe direction, stop
            self.get_logger().warn(f'Direction blocked: {["LEFT","FORWARD","RIGHT"][direction]}, stopping.')
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
