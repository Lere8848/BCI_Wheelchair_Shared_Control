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
        
        # 订阅多路径检测信号
        self.multipath_sub = self.create_subscription(
            Int8MultiArray, 
            '/multipath_detected', 
            self.multipath_callback, 
            10
        )

        # 订阅路径阻塞信号
        self.path_blocked_sub = self.create_subscription(Bool, '/path_blocked', self.path_blocked_callback, 10)

        self.danger = False
        self.create_subscription(Bool, '/danger_stop', self.danger_callback, 10)

        self.path_options = [1, 1, 1]  # default all passable
        self.path_blocked = False      # 前方路径是否被阻塞
        self.auto_cmd = Twist()        # 存储自动规划的命令
        
        # 多路径检测状态
        self.multipath_detected = False
        self.multipath_status = [False, False, False]
        
        # 控制模式状态机
        self.control_mode = "USER_INPUT"  # USER_INPUT, POTENTIAL_FIELD, WAITING
        self.auto_mode = True          # 默认启用自动模式
        
        # 势场运行相关参数
        self.potential_field_start_time = None  # 势场模式开始时间
        self.potential_field_duration = 3.0    # 势场运行持续时间(秒)
        
        self.cmd_active = False        # 用户命令是否激活
        self.last_cmd_time = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def path_callback(self, msg):
        if len(msg.data) == 3:
            self.path_options = msg.data

    def multipath_callback(self, msg):
        """接收多路径检测结果"""
        if len(msg.data) == 3:
            self.multipath_status = [bool(x) for x in msg.data]
            self.multipath_detected = sum(self.multipath_status) >= 2
            
            if self.multipath_detected and self.control_mode == "POTENTIAL_FIELD":
                # 在势场模式下检测到多路径，立刻停止并切换到用户输入模式
                self.get_logger().warn('Multiple paths detected! Switching to user input mode.')
                self.control_mode = "USER_INPUT"
                self.cmd_active = False
                self.auto_mode = False
                # 立即停止
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)

    def danger_callback(self, msg):
        """危险检测回调 - 只在极端情况下触发，不阻挡势场前进"""
        self.danger = msg.data
        if self.danger:
            if self.control_mode == "POTENTIAL_FIELD":
                # 在势场模式下触发危险停止，切换回用户输入模式
                self.get_logger().warn('Danger detected! Stopping potential field navigation, waiting for user to re-enter intent.')
                self.control_mode = "USER_INPUT"
                self.cmd_active = False
                self.auto_mode = False
            else:
                # 在其他模式下的危险处理
                self.get_logger().warn('Danger stop triggered! Emergency braking!')
                self.cmd_active = False
                self.auto_mode = False
            
            # 立即停止轮椅
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
    
    def auto_cmd_callback(self, msg):
        # 保存自动规划的命令，供后续使用
        self.auto_cmd = msg
        
    def path_blocked_callback(self, msg):
        # 更新路径阻塞状态
        self.path_blocked = msg.data

    def timer_callback(self):
        """主控制循环 - 实现状态机逻辑"""
        current_time = self.get_clock().now()
        
        # 状态机逻辑
        if self.control_mode == "USER_INPUT":
            # 用户输入模式：等待用户命令或执行用户命令
            if self.cmd_active and not self.danger:
                # 检查用户命令是否过期
                elapsed = current_time - self.last_cmd_time
                if elapsed > Duration(seconds=1.5):  # 用户意图持续1.5秒
                    self.get_logger().info('User intent expired, switching to potential field mode')
                    self.control_mode = "POTENTIAL_FIELD"
                    self.potential_field_start_time = current_time
                    self.cmd_active = False
                    self.auto_mode = True
            
        elif self.control_mode == "POTENTIAL_FIELD":
            # 势场模式：运行3秒或直到满足退出条件
            if not self.danger and self.auto_mode:
                # 检查是否运行满3秒
                if self.potential_field_start_time is not None:
                    elapsed = current_time - self.potential_field_start_time
                    if elapsed > Duration(seconds=self.potential_field_duration):
                        # 3秒时间到，切换回用户输入模式
                        self.get_logger().info('Potential field mode 3 seconds completed, switching back to user input mode')
                        self.control_mode = "USER_INPUT"
                        self.auto_mode = False
                        # 停止运动，等待用户输入
                        stop_cmd = Twist()
                        self.cmd_pub.publish(stop_cmd)
                        return
                
                # 发布自动规划的命令
                if not (self.auto_cmd.linear.x == 0.0 and self.auto_cmd.angular.z == 0.0):
                    self.get_logger().debug(f'Potential field navigation: linear.x={self.auto_cmd.linear.x:.2f}, angular.z={self.auto_cmd.angular.z:.2f}')
                    self.cmd_pub.publish(self.auto_cmd)
                else:
                    # 如果势场算法输出为零，可能到达目标或遇到问题，切换回用户模式
                    self.get_logger().info('Potential field output is zero, switching back to user input mode')
                    self.control_mode = "USER_INPUT"
                    self.auto_mode = False

    def user_cmd_callback(self, msg):
        """用户命令回调 - 在用户输入模式下执行用户意图"""
        direction = msg.data  # 0=left, 1=forward, 2=right

        if direction < 0 or direction > 2:
            self.get_logger().warn(f'Invalid direction: {direction}')
            return

        # 只在用户输入模式下处理用户命令
        if self.control_mode != "USER_INPUT":
            self.get_logger().debug(f'Current mode is {self.control_mode}, ignoring user input')
            return

        twist = Twist()

        # 强制切换到用户输入模式
        self.control_mode = "USER_INPUT"
        self.auto_mode = False

        if self.path_options[direction] == 1:
            # 方向安全，生成运动命令
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
            self.get_logger().info(f'Executing user intent: {["Left", "Forward", "Right"][direction]}')

            # 设置命令激活并记录时间
            self.cmd_active = True
            self.last_cmd_time = self.get_clock().now()

        else:
            # 不安全的方向，停止
            self.get_logger().warn(f'Blocked direction: {["Left", "Forward", "Right"][direction]} - Stopping movement')
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
