# fusion_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int8MultiArray, Bool
from geometry_msgs.msg import Twist
import time
import threading
from rclpy.duration import Duration

# LSL导入
try:
    from pylsl import StreamInfo, StreamOutlet
    LSL_AVAILABLE = True
except ImportError:
    LSL_AVAILABLE = False

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.user_cmd_sub = self.create_subscription(Int8, '/user_cmd', self.user_cmd_callback, 10)
        self.path_opt_sub = self.create_subscription(Int8MultiArray, '/path_options', self.path_callback, 10)
        
        # 订阅Ground Truth输入
        self.gt_input_sub = self.create_subscription(Int8, '/gt_input', self.gt_input_callback, 10)
        
        # 订阅势场算法的路径指令（按需生成）
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

        self.path_options = [1, 1, 1]  # 路径可行性 [左, 前, 右]
        self.path_blocked = False      # 整体路径是否被阻塞
        self.auto_cmd = Twist()        # 势场算法生成的运动命令
        
        # 多路径检测状态
        self.multipath_detected = False
        self.multipath_status = [False, False, False]  # [左, 前, 右] 哪些方向有路径
        
        # 控制状态：默认静止，等待用户触发
        self.state = "IDLE"  # IDLE, EXECUTING, WAITING_FOR_USER
        self.execution_start_time = None
        self.execution_duration = 3.0  # 每次执行持续时间(秒)
        
        # 用户意图状态
        self.pending_user_direction = None  # 待执行的用户方向意图
        self.last_user_command_time = None
        
        # Ground Truth 状态控制
        self.waiting_for_groundtruth = False  # 是否等待真实意图输入
        self.groundtruth_received = False     # 是否已接收真实意图
        
        # LSL输出流设置 - 发送轮椅运动状态给BCI
        self.lsl_outlet = None
        self.wheelchair_moving = False  # 轮椅运动状态跟踪
        if LSL_AVAILABLE:
            try:
                # 创建LSL流：2个通道 [Moving_Flag, Stop_Flag]
                info = StreamInfo('Wheelchair_Motion', 'Motion_Flag', 2, 10, 'float32', 'control_fusion_node')
                self.lsl_outlet = StreamOutlet(info)
                self.get_logger().info('LSL outlet created for wheelchair motion status transmission to BCI')
            except Exception as e:
                self.get_logger().warn(f'Failed to create LSL outlet: {str(e)}')
        else:
            self.get_logger().warn('LSL not available - wheelchair motion status will not be sent to BCI')
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def path_callback(self, msg):
        if len(msg.data) == 3:
            self.path_options = msg.data

    def multipath_callback(self, msg):
        """接收多路径检测结果"""
        if len(msg.data) == 3:
            self.multipath_status = [bool(x) for x in msg.data]
            self.multipath_detected = sum(self.multipath_status) >= 2
            
            if self.multipath_detected:
                self.get_logger().info(f'Multiple paths detected: Left={self.multipath_status[0]}, Front={self.multipath_status[1]}, Right={self.multipath_status[2]}')
                # 在多路径情况下，如果没有待执行的用户指令，提示等待
                if self.pending_user_direction is None and self.state == "IDLE":
                    self.get_logger().warn('Multiple paths available, waiting for user direction input...')

    def danger_callback(self, msg):
        """危险检测回调 - 立即停止"""
        self.danger = msg.data
        if self.danger:
            self.get_logger().warn('Danger detected! Emergency stop!')
            # 立即停止轮椅
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
    
    def auto_cmd_callback(self, msg):
        """接收势场算法生成的运动命令"""
        self.auto_cmd = msg
        
    def path_blocked_callback(self, msg):
        """更新路径阻塞状态"""
        self.path_blocked = msg.data

    def gt_input_callback(self, msg):
        """Ground Truth输入回调"""
        if self.waiting_for_groundtruth:
            direction_names = ["Left", "Forward", "Right"]
            self.get_logger().info(f'Ground truth received: {direction_names[msg.data]}')
            self.groundtruth_received = True
            self.waiting_for_groundtruth = False

    def send_motion_status(self, is_moving):
        """发送轮椅运动状态到BCI系统"""
        if self.lsl_outlet and LSL_AVAILABLE:
            try:
                # [Moving_Flag, Stop_Flag] - 1表示状态激活，0表示状态未激活
                moving_flag = 1.0 if is_moving else 0.0
                stop_flag = 0.0 if is_moving else 1.0
                motion_status = [moving_flag, stop_flag]
                self.lsl_outlet.push_sample(motion_status)
                
                # 仅在状态变化时记录日志
                if self.wheelchair_moving != is_moving:
                    self.get_logger().info(f'Wheelchair motion status sent to BCI: Moving={is_moving}')
                    self.wheelchair_moving = is_moving
            except Exception as e:
                self.get_logger().warn(f'Failed to send motion status via LSL: {str(e)}')

    def timer_callback(self):
        """主控制循环 - 用户触发的势场执行"""
        current_time = self.get_clock().now()
        
        # 检查危险状态
        if self.danger:
            # 危险状态下立即停止并回到空闲状态
            self.state = "IDLE"
            self.pending_user_direction = None
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.send_motion_status(False)  # 发送紧急停止状态
            return
        
        # 状态机逻辑
        if self.state == "IDLE":
            # 空闲状态：轮椅静止，等待用户触发
            if self.pending_user_direction is not None:
                # 检查是否等待ground truth
                if self.waiting_for_groundtruth:
                    # 等待ground truth输入，保持静止
                    stop_cmd = Twist()
                    self.cmd_pub.publish(stop_cmd)
                    self.send_motion_status(False)
                    return
                
                # Ground truth已接收或不需要等待，检查路径可行性
                direction = self.pending_user_direction
                if self.path_options[direction] == 1:
                    self.get_logger().info(f'Starting execution for user direction: {["Left", "Forward", "Right"][direction]}')
                    self.state = "EXECUTING"
                    self.execution_start_time = current_time
                    self.send_motion_status(True)  # 发送运动开始状态
                    # 清除待执行意图
                    self.pending_user_direction = None
                else:
                    # 用户选择的方向被阻塞
                    self.get_logger().warn(f'User selected direction {["Left", "Forward", "Right"][direction]} is blocked, staying idle')
                    self.pending_user_direction = None
                    stop_cmd = Twist()
                    self.cmd_pub.publish(stop_cmd)
                    self.send_motion_status(False)  # 发送停止状态
            else:
                # 没有待执行意图，保持静止
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                self.send_motion_status(False)  # 发送停止状态
        
        elif self.state == "EXECUTING":
            # 执行状态：按照势场算法执行用户选择的方向
            if self.execution_start_time is not None:
                elapsed = current_time - self.execution_start_time
                
                if elapsed > Duration(seconds=self.execution_duration):
                    # 执行时间到达，回到空闲状态
                    self.get_logger().info('Execution completed, returning to idle state')
                    self.state = "IDLE"
                    self.execution_start_time = None
                    stop_cmd = Twist()
                    self.cmd_pub.publish(stop_cmd)
                    self.send_motion_status(False)  # 发送停止状态
                    return
            
            # 继续执行势场算法的输出
            if not (self.auto_cmd.linear.x == 0.0 and self.auto_cmd.angular.z == 0.0):
                self.get_logger().debug(f'Executing potential field command: linear={self.auto_cmd.linear.x:.2f}, angular={self.auto_cmd.angular.z:.2f}')
                self.cmd_pub.publish(self.auto_cmd)
            else:
                # 势场算法输出为零，可能到达目标或遇到障碍，提前结束执行
                self.get_logger().info('Potential field output is zero, ending execution early')
                self.state = "IDLE"
                self.execution_start_time = None
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                self.send_motion_status(False)  # 发送停止状态
    
    def user_cmd_callback(self, msg):
        """用户命令回调 - 触发势场算法执行用户选择的方向"""
        direction = msg.data  # 0=left, 1=forward, 2=right

        if direction < 0 or direction > 2:
            self.get_logger().warn(f'Invalid direction: {direction}')
            return

        direction_names = ["Left", "Forward", "Right"]
        self.get_logger().info(f'User command received: {direction_names[direction]}')
        
        # 如果当前正在执行，终止当前执行并准备新的执行
        if self.state == "EXECUTING":
            self.get_logger().info('Interrupting current execution for new user command')
            self.state = "IDLE"
            self.execution_start_time = None
            self.send_motion_status(False)  # 发送停止状态
        
        # 等待Ground Truth输入
        self.get_logger().info('Waiting for ground truth input...')
        self.waiting_for_groundtruth = True
        self.groundtruth_received = False
        self.pending_user_direction = direction
        self.last_user_command_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
