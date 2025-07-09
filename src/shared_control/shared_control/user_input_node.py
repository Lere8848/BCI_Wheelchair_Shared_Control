# user_input_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys
import platform

if platform.system() == 'Windows':
    import msvcrt
else:
    import termios
    import tty
    import select

class UserInputNode(Node):
    def __init__(self):
        super().__init__('user_input_node')
        self.pub = self.create_publisher(Int8, '/user_cmd', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Press a/w/d for Left/Forward/Right')

    def get_key(self):
        if platform.system() == 'Windows':
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8')
            else:
                return ''
        else:
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            key = sys.stdin.read(1) if rlist else ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

    def timer_callback(self):
        key = self.get_key()
        msg = Int8()

        if key == 'a':
            msg.data = 0  # left
            self.pub.publish(msg)
            self.get_logger().info('Intent: LEFT')
        elif key == 'w':
            msg.data = 1  # forward
            self.pub.publish(msg)
            self.get_logger().info('Intent: FORWARD')
        elif key == 'd':
            msg.data = 2  # right
            self.pub.publish(msg)
            self.get_logger().info('Intent: RIGHT')

def main(args=None):
    rclpy.init(args=args)
    node = UserInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
