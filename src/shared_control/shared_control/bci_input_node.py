# bci_input_node.py
# 必须要BCI那边先送数据 这边再开节点才能收到 时间关系 现不深究 后面有时间看看怎么优化
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32MultiArray
import numpy as np
import time
import threading

# LSL库导入 - 用于接收BCI数据
try:
    from pylsl import StreamInlet, resolve_streams
    LSL_AVAILABLE = True
except ImportError:
    LSL_AVAILABLE = False

class BCIInputNode(Node):
    def __init__(self):
        super().__init__('bci_input_node')
        
        # 发布器设置
        self.user_cmd_pub = self.create_publisher(Int8, '/user_cmd', 10)  # 与原user_input_node兼容
        self.bci_info_pub = self.create_publisher(Float32MultiArray, '/bci_info', 10)  # 新增：发布BCI信息给Unity
        
        # BCI数据状态
        self.latest_action = 1      # gt字段：实际去的方向 (0=左, 1=前, 2=右)
        self.latest_confidences = [0.0, 0.0, 0.0]  # 三个方向的置信度 [左, 前, 右]
        self.latest_valid = 0       # valid字段：指令是否执行 (1=执行, 0=不执行)
        
        # LSL数据流相关
        self.inlet = None
        self.lsl_connected = False
        
        # 定时器 - 定期发布BCI信息给Unity
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 启动LSL接收线程
        if LSL_AVAILABLE:
            self.lsl_thread = threading.Thread(target=self.lsl_receiver_thread, daemon=True)
            self.lsl_thread.start()
            self.get_logger().info('BCI Input Node initialized with LSL support')
        else:
            self.get_logger().warn('LSL library not available - BCI node cannot function without LSL')
            # 在没有LSL的情况下，发布默认数据以保持系统运行
            self.latest_action = 1  # 默认前进
            self.latest_confidences = [0.0, 0.0, 0.0]  # 零置信度
            self.latest_valid = 0  # 不执行

    def lsl_receiver_thread(self):
        """LSL数据接收线程"""
        try:
            # 查找BCI数据流
            self.get_logger().info('Looking for BCI data stream...')
            streams = resolve_streams()
            
            # 过滤出BCI_Commands类型的流
            bci_streams = [s for s in streams if s.type() == 'BCI_Commands'] # 与Yuze对接
            
            if not bci_streams:
                self.get_logger().warn('No BCI data stream found, waiting for stream...')
                # 等待并重试
                time.sleep(2.0)
                return
            
            # 连接到第一个找到的流
            self.inlet = StreamInlet(bci_streams[0])
            self.lsl_connected = True
            self.get_logger().info('Connected to BCI data stream')
            
            # 持续接收数据
            while rclpy.ok() and self.lsl_connected:
                try:
                    # 接收数据格式: dict{0:conf, 1:conf, 2:conf, gt:0/1/2, valid:1/0}
                    sample, timestamp = self.inlet.pull_sample(timeout=1.0)
                    
                    if sample is not None and len(sample) >= 5:
                        # 解析字典格式数据
                        self.latest_confidences = [
                            float(sample[0]),  # 左方向置信度
                            float(sample[1]),  # 前方向置信度  
                            float(sample[2])   # 右方向置信度
                        ]
                        self.latest_action = int(sample[3])    # gt字段：实际方向
                        self.latest_valid = int(sample[4])     # valid字段：是否执行
                        
                        # 如果valid为1，发布用户指令
                        if self.latest_valid == 1:
                            self.publish_user_command()
                            
                except Exception as e:
                    self.get_logger().warn(f'LSL data reception error: {str(e)}')
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f'LSL connection error: {str(e)}')
            self.lsl_connected = False

    def publish_user_command(self):
        """发布用户指令到ROS系统"""
        if self.latest_action in [0, 1, 2]:
            msg = Int8()
            msg.data = self.latest_action
            self.user_cmd_pub.publish(msg)
            
            direction_names = ['LEFT', 'FORWARD', 'RIGHT']
            selected_confidence = self.latest_confidences[self.latest_action]
            self.get_logger().info(
                f'BCI Intent: {direction_names[self.latest_action]} '
                f'(confidence: {selected_confidence:.3f})'
            )

    def timer_callback(self):
        """定时发布BCI信息给Unity端"""
        # 创建包含BCI状态信息的消息
        # 发送数据格式: [conf_left, conf_forward, conf_right, gt, valid, connected, threshold]
        bci_info_msg = Float32MultiArray()
        bci_info_msg.data = [
            self.latest_confidences[0],     # 左方向置信度
            self.latest_confidences[1],     # 前方向置信度
            self.latest_confidences[2],     # 右方向置信度
            float(self.latest_action),      # gt：实际选择的方向
            float(self.latest_valid),       # valid：是否执行
            1.0 if self.lsl_connected else 0.0,  # 连接状态
            1.0 # 置信度判断threshold 先占位 看后面要不要
        ]
        self.bci_info_pub.publish(bci_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BCIInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
