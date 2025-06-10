import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import platform

# 判断操作系统类型
if platform.system() == 'Windows':
    import msvcrt
else: # Linux
    import termios
    import tty
    import select

class SimpleSharedControl(Node):
    def __init__(self):
        super().__init__('simple_shared_control')
        # 创建发布者，发布Twist消息到/cmd_vel话题
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 创建定时器，每0.1秒调用一次timer_callback
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.manual_control = False  # 是否为手动控制
        self.last_cmd = Twist()      # 上一次的指令
        self.get_logger().info('SimpleSharedControl node initialized.')

    def get_key(self):
        # 获取键盘输入
        if platform.system() == 'Windows':
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8')
                print(f'[DEBUG] Key pressed: {key}')
                return key
            else:
                return ''
        else:
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

    def timer_callback(self):
        # 定时器回调函数，处理键盘输入并发布消息
        key = self.get_key()
        cmd = Twist()

        if key == 'w':
            cmd.linear.x = 1.0      # 前进
            self.manual_control = True
            self.get_logger().info('[DEBUG] Forward command issued.')
        elif key == 's':
            cmd.linear.x = -1.0     # 后退
            self.manual_control = True
            self.get_logger().info('[DEBUG] Backward command issued.')
        elif key == 'a':
            cmd.angular.z = 1.0     # 左转
            self.manual_control = True
            self.get_logger().info('[DEBUG] Turn left command issued.')
        elif key == 'd':
            cmd.angular.z = -1.0    # 右转
            self.manual_control = True
            self.get_logger().info('[DEBUG] Turn right command issued.')
        elif key == 'q':
            self.get_logger().info('Exiting...')
            rclpy.shutdown()
            return
        else:
            if self.manual_control:
                cmd = self.last_cmd  # 沿用上一次指令
                self.get_logger().info('[DEBUG] Continuing last manual command.')
            else:
                cmd.linear.x = 0.3   # 自动前进
                self.get_logger().info('[DEBUG] Automatic forward command issued.')

        self.get_logger().info(f'[DEBUG] Publishing cmd: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')
        self.pub.publish(cmd)        # 发布消息
        self.last_cmd = cmd          # 保存本次指令

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSharedControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
