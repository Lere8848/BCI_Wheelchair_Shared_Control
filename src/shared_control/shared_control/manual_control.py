import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import platform

if platform.system() == 'Windows':
    import msvcrt
else:
    import termios
    import tty
    import select

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('ManualControlNode initialized.')

    def get_key(self):
        if platform.system() == 'Windows':
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8')
                return key
            else:
                return ''
        else:
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

    def timer_callback(self):
        key = self.get_key()
        cmd = Twist()

        if key == 'w':
            cmd.linear.x = 1.0
        elif key == 's':
            cmd.linear.x = -1.0
        elif key == 'a':
            cmd.angular.z = 1.0
        elif key == 'd':
            cmd.angular.z = -1.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()