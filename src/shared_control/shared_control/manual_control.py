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
        self.get_logger().info('Press "w", "s", "a", "d" to control the robot. Press "q" to quit.')

    def get_key(self):
        if platform.system() == 'Windows':
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8')
                self.get_logger().debug(f'[DEBUG] Key pressed: {key}')
                return key
            else:
                return ''
        else:
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist:
                key = sys.stdin.read(1)
                self.get_logger().debug(f'[DEBUG] Key pressed: {key}')
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

    def timer_callback(self):
        key = self.get_key()
        cmd = Twist()

        if key == 'w':
            cmd.linear.x = 0.5
            self.get_logger().info('Moving forward')
        elif key == 's':
            cmd.linear.x = -0.4
            self.get_logger().info('Moving backward')
        elif key == 'a':
            cmd.angular.z = -0.4
            self.get_logger().info('Turning left')
        elif key == 'd':
            cmd.angular.z = 0.5
            self.get_logger().info('Turning right')
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if key != '':
                self.get_logger().info(f'Unknown key: {key}')

        # self.get_logger().info(f'[DEBUG] Publishing cmd: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()