# fusion_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int8MultiArray, Bool
from geometry_msgs.msg import Twist
import time
from rclpy.duration import Duration

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.user_cmd_sub = self.create_subscription(Int8, '/user_cmd', self.user_cmd_callback, 10)
        self.path_opt_sub = self.create_subscription(Int8MultiArray, '/path_options', self.path_callback, 10)
        
        # 订阅自动规划的路径指令
        self.auto_cmd_sub = self.create_subscription(Twist, '/auto_cmd_vel', self.auto_cmd_callback, 10)
        
        # 订阅路径阻塞信号
        self.path_blocked_sub = self.create_subscription(Bool, '/path_blocked', self.path_blocked_callback, 10)

        self.danger = False
        self.create_subscription(Bool, '/danger_stop', self.danger_callback, 10)

        self.path_options = [1, 1, 1]  # default all passable
        self.path_blocked = False      # 前方路径是否被阻塞
        self.auto_cmd = Twist()        # 存储自动规划的命令
        
        # 控制模式
        self.auto_mode = True          # 默认启用自动模式
        
        self.cmd_active = False        # 用户命令是否激活
        self.last_cmd_time = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def path_callback(self, msg):
        if len(msg.data) == 3:
            self.path_options = msg.data

    def danger_callback(self, msg):
        self.danger = msg.data
        if self.danger:
            self.get_logger().warn('DANGER STOP TRIGGERED. Emergency brake!')
            self.cmd_pub.publish(Twist())
            self.cmd_active = False # Stop the wheelchair immediately
            self.auto_mode = False  # 危险情况下也停止自动模式
    
    def auto_cmd_callback(self, msg):
        # 保存自动规划的命令，供后续使用
        self.auto_cmd = msg
        
    def path_blocked_callback(self, msg):
        # 更新路径阻塞状态
        self.path_blocked = msg.data

    def timer_callback(self):
        # 检查用户命令是否过期
        if self.cmd_active and not self.danger:
            elapsed = self.get_clock().now() - self.last_cmd_time
            if elapsed > Duration(seconds=1.5): # 保证用户意图持续1.5秒
                self.get_logger().info('user command expired, switching to auto mode.')
                self.cmd_active = False
                # 用户命令结束后恢复自动模式
                self.auto_mode = True
        
        # 如果没有用户命令且不处于危险状态，切换到自动模式
        if not self.cmd_active and not self.danger and self.auto_mode:
            # 检查是否有自动规划的命令
            if not (self.auto_cmd.linear.x == 0.0 and self.auto_cmd.angular.z == 0.0):
                self.get_logger().debug(f'automatic path: linear.x={self.auto_cmd.linear.x:.2f}, angular.z={self.auto_cmd.angular.z:.2f}')
                self.cmd_pub.publish(self.auto_cmd)
            elif self.path_blocked:
                self.get_logger().warn('obstacle detected, waiting for user input')
                # 路径被阻塞时停止，等待用户输入
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)

    def user_cmd_callback(self, msg):
        direction = msg.data  # 0=left, 1=forward, 2=right

        twist = Twist()

        if direction < 0 or direction > 2:
            self.get_logger().warn(f'invalid direction: {direction}')
            return

        # 收到用户命令时，暂时切换到手动模式
        self.auto_mode = False

        if self.path_options[direction] == 1:
            # Direction is safe, generate movement - 确保前进+转向组合运动
            if direction == 0:  # left - 缓慢前进并左转
                twist.linear.x = 0.34   # 缓慢前进分量
                twist.angular.z = -0.15  # 左转分量
            elif direction == 1:  # forward - 正常前进
                twist.linear.x = 0.5   # 正常前进速度
                twist.angular.z = 0.0  # 无转向
            elif direction == 2:  # right - 缓慢前进并右转
                twist.linear.x = 0.34   # 缓慢前进分量
                twist.angular.z = 0.15  # 右转分量

            self.cmd_pub.publish(twist)
            self.get_logger().info(f'execute user intent: {["left", "forward", "right"][direction]}')

            # 设置命令激活并记录时间
            self.cmd_active = True
            self.last_cmd_time = self.get_clock().now()

        else:
            # 不安全的方向，停止
            self.get_logger().warn(f'direction blocked: {["left", "forward", "right"][direction]}, stopping.')
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
