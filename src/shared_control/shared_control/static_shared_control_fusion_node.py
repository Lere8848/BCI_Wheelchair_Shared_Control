# fusion_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int8MultiArray, Bool
from geometry_msgs.msg import Twist
import time

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.user_cmd_sub = self.create_subscription(Int8, '/user_cmd', self.user_cmd_callback, 10)
        self.path_opt_sub = self.create_subscription(Int8MultiArray, '/path_options', self.path_callback, 10)

        self.danger = False
        self.create_subscription(Bool, '/danger_stop', self.danger_callback, 10)

        self.path_options = [1, 1, 1]  # default all passable

    def path_callback(self, msg):
        if len(msg.data) == 3:
            self.path_options = msg.data

    def danger_callback(self, msg):
        self.danger = msg.data
        if self.danger:
            self.get_logger().warn('DANGER STOP TRIGGERED. Emergency brake!')
            self.cmd_pub.publish(Twist()) # Stop the wheelchair immediately

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
                twist.linear.x = 0.5
            elif direction == 2:  # right
                twist.angular.z = 0.4
            
            self.cmd_pub.publish(twist)
            self.get_logger().info(f'Executing user intent: {["LEFT","FORWARD","RIGHT"][direction]}')
        
            # 每次控制持续1.5秒 延迟后发送停止
            time.sleep(1.5)
            self.get_logger().info('Executing finished')
            stop_twist = Twist()
            self.cmd_pub.publish(stop_twist)

        else:
            # Unsafe direction, stop
            self.get_logger().warn(f'Direction blocked: {["LEFT","FORWARD","RIGHT"][direction]}, stopping.')
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
