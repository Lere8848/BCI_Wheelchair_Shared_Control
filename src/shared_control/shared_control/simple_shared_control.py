import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import termios
import tty
import select

class SimpleSharedControl(Node):
    def __init__(self):
        super().__init__('simple_shared_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)  # 创建/cmd_vel话题的发布者
        self.timer = self.create_timer(0.1, self.timer_callback)  # 创建定时器，周期0.1秒
        self.manual_control = False  # 是否为手动控制
        self.last_cmd = Twist()  # 上一次的控制指令

    def get_key(self):
        # 获取键盘输入（非阻塞）
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)  # 读取一个字符
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def timer_callback(self):
        # 定时器回调函数
        key = self.get_key()
        cmd = Twist()

        if key == 'w':
            cmd.linear.x = 1.0  # 前进
            self.manual_control = True
        elif key == 's':
            cmd.linear.x = -1.0  # 后退
            self.manual_control = True
        elif key == 'a':
            cmd.angular.z = 1.0  # 左转
            self.manual_control = True
        elif key == 'd':
            cmd.angular.z = -1.0  # 右转
            self.manual_control = True
        elif key == 'q':
            self.get_logger().info('Exiting...')  # 退出
            rclpy.shutdown()
            return
        else:
            if self.manual_control:
                cmd = self.last_cmd  # 停止时保持上一次动作
            else:
                cmd.linear.x = 0.3  # 自动直行

        self.pub.publish(cmd)  # 发布控制指令
        self.last_cmd = cmd  # 保存本次指令

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2
    node = SimpleSharedControl()  # 创建节点
    rclpy.spin(node)  # 循环等待回调
    node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2
