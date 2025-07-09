import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys
import platform

if platform.system() == 'Windows':
    import msvcrt
else:
    import termios
    import tty
    import select

class SimpleSharedControl(Node):
    def __init__(self):
        super().__init__('simple_shared_control_demo')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.latest_scan = None
        self.get_logger().info('SimpleSharedControl node initialized with LIDAR safety check.')

    # (TODO 后续可以放到 utils.py 中)
    def get_key(self):
        if platform.system() == 'Windows': # Windows-specific key reading
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8')
                print(f'[DEBUG] Key pressed: {key}')
                return key
            else:
                return ''
        else: # Unix-like systems
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist:
                key = sys.stdin.read(1)
                print(f'[DEBUG] Key pressed: {key}')
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def is_obstacle_too_close(self, threshold=1.0, center_angle_width_deg=20): # 1m thres
        if self.latest_scan is None:
            return False

        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        center_index = len(self.latest_scan.ranges) // 2
        half_span = int((center_angle_width_deg * 3.1416 / 180) / angle_increment)

        start = max(0, center_index - half_span)
        end = min(len(self.latest_scan.ranges), center_index + half_span)

        for i in range(start, end):
            if self.latest_scan.ranges[i] < threshold:
                return True

        return False

    def timer_callback(self):
        key = self.get_key()
        cmd = Twist()

        too_close = self.is_obstacle_too_close()

        if too_close:
            self.get_logger().warn('[SAFETY] Obstacle too close! Forcing backward.')
            cmd.linear.x = -0.3  # 强制后退
        elif key == 'w':
            cmd.linear.x = 0.5
        elif key == 's':
            cmd.linear.x = -0.3
        elif key == 'a':
            cmd.angular.z = -0.3
        elif key == 'd':
            cmd.angular.z = 0.3
        elif key == 'q':
            self.get_logger().info('Exiting...')
            rclpy.shutdown()
            return
        else:
            cmd.linear.x = 0.3

        self.get_logger().info(f'[DEBUG] Publishing cmd: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSharedControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
