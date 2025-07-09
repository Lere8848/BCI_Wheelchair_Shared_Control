# path_eval_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Int8MultiArray, Bool
import numpy as np

class PathEvalNode(Node):
    def __init__(self):
        super().__init__('path_eval_node')
        self.pub = self.create_publisher(Int8MultiArray, '/path_options', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # self.sub = self.create_subscription(Range, '/ultrasonic_front', self.lidar_callback, 10)
        self.danger_pub = self.create_publisher(Bool, '/danger_stop', 10)
        self.lidar_ranges = []
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('PathEvalNode initialized with LIDAR.')

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)
        # self.get_logger().info(f'LIDAR data received: {len(self.lidar_ranges)} ranges')

    def timer_callback(self):
        if len(self.lidar_ranges) == 0:
            return

        # lidar ranges 分为左、中、右三部分
        # 假设 lidar_ranges 长度为 360，左 120，前 120，右 120
        total_rays = len(self.lidar_ranges)
        left_idx = range(0, total_rays // 3)
        forward_idx = range(total_rays // 3, 2 * total_rays // 3)
        right_idx = range(2 * total_rays // 3, total_rays)

        # ultrasonic 冗余检测 防止 LIDAR 数据不全
        # if self.front_ultrasonic < 0.5:
        #    path[1] = 0 

        def is_open(idx_range, threshold):
            distances = np.array(self.lidar_ranges[idx_range])
            valid = distances[~np.isnan(distances)]
            if len(valid) < 5:
                return False
            return np.percentile(distances, 40) > threshold # 30% percentile is a good indicator of openness
            # 记得寻找文献证明这一点

        path = [
            int(is_open(left_idx, 0.6)), # left
            int(is_open(forward_idx, 0.6)), # forward
            int(is_open(right_idx, 0.6)) # right
        ]

        all_dists = np.array(self.lidar_ranges)
        danger = np.any(all_dists < 0.6) # 如果有任意距离小于0.6m，则认为有危险

        # path_options
        path_msg = Int8MultiArray()
        path_msg.data = path
        self.pub.publish(path_msg)

        # danger_stop
        danger_msg = Bool()
        danger_msg.data = bool(danger) # numpt的bool和ros2 msg bool 不兼容 需要转换
        self.danger_pub.publish(danger_msg)

        self.get_logger().info(f'/path_options: {path} | danger_stop: {danger}')

def main(args=None):
    rclpy.init(args=args)
    node = PathEvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
