# path_eval_node.py
# 路径评估节点：处理来自Unity仿真环境的激光雷达数据
# 
# 重要说明：
# - 激光雷达数据来源：Unity仿真环境
# - 坐标系转换：Unity坐标系 -> ROS坐标系
# - 方向定义：需要根据Unity的坐标系正确映射左右方向
#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Int8MultiArray, Bool
import numpy as np
import math

class PathEvalNode(Node):
    def __init__(self):
        super().__init__('path_eval_node')
        
        # 发布订阅
        self.pub = self.create_publisher(Int8MultiArray, '/path_options', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.danger_pub = self.create_publisher(Bool, '/danger_stop', 10)
        
        # 新增发布话题
        self.path_blocked_pub = self.create_publisher(Bool, '/path_blocked', 10)
        self.multipath_pub = self.create_publisher(Int8MultiArray, '/multipath_detected', 10)
        
        # 数据存储
        self.lidar_ranges = []
        self.laser_data = None
        
        # 滑动窗口障碍物检测参数
        self.window_size = 5
        self.min_consecutive_detections = 2
        self.obstacle_detection_history = []
        
        # 检测参数
        self.path_detection_distance = 2.0 # 路径检测距离(m)
        self.path_width_threshold = 1.0 # 路径宽度阈值(m)
        self.wall_detection_distance = 0.5 # 墙壁检测距离(m)
        self.min_obstacle_dist = 0.5 # 最小障碍物距离(m)

        self.timer = self.create_timer(0.2, self.timer_callback)  # 提高频率
        self.get_logger().info('PathEvalNode initialized - Processing LIDAR data from Unity simulation environment')

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)
        self.laser_data = msg  # 保存完整的激光数据
        # self.get_logger().info(f'LIDAR data received: {len(self.lidar_ranges)} ranges')
    
    # def ultrasonic_callback(self, msg):
    #     self.front_ultrasonic = msg.range

    def analyze_sectors_openness(self, ranges, angle_min, angle_increment):
        """
        分析三大扇区的开放程度（从potential_field_planner移植的高级方法）
        返回: dict {方向: {小扇区开放度列表}}
        """
        # 定义三个主扇区：与potential_field_planner一致的Unity坐标系
        major_sectors = {
            'left': (-math.pi/2, -math.pi/6),    # 左扇区：-90°到-30°
            'front': (-math.pi/6, math.pi/6),    # 前扇区：-30°到30°
            'right': (math.pi/6, math.pi/2)      # 右扇区：30°到90°
        }
        
        sub_sectors_per_major = 6  # 每个主扇区分为6个子扇区
        sector_analysis = {}
        
        for direction, (start_angle, end_angle) in major_sectors.items():
            sector_width = (end_angle - start_angle) / sub_sectors_per_major
            sub_sector_openness = []
            
            # 分析每个小扇区
            for i in range(sub_sectors_per_major):
                sub_start = start_angle + i * sector_width
                sub_end = start_angle + (i + 1) * sector_width
                sub_center = (sub_start + sub_end) / 2
                
                # 计算该小扇区的开放度
                openness = self.calculate_sub_sector_openness(
                    ranges, angle_min, angle_increment, 
                    sub_start, sub_end, sub_center
                )
                sub_sector_openness.append({
                    'center_angle': sub_center,
                    'openness': openness,
                    'start_angle': sub_start,
                    'end_angle': sub_end
                })
            
            sector_analysis[direction] = sub_sector_openness
        
        return sector_analysis

    def calculate_sub_sector_openness(self, ranges, angle_min, angle_increment, 
                                    start_angle, end_angle, center_angle):
        """
        计算小扇区的开放程度（从potential_field_planner移植的高级方法）
        """
        distances_in_sector = []
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
                
            angle = angle_min + i * angle_increment
            
            # 检查是否在当前小扇区范围内
            if start_angle <= angle <= end_angle:
                distances_in_sector.append(dist)
        
        if len(distances_in_sector) == 0:
            return 0.0  # 无数据时返回不可通行
        
        distances = np.array(distances_in_sector)
        
        # 开放度计算
        avg_distance = np.mean(distances)
        min_distance = np.min(distances)
        
        # 基础开放度：基于平均距离
        base_openness = min(1.0, avg_distance / 3.0)
        
        # 安全性惩罚：如果最小距离太小，大幅降低开放度
        if min_distance < 0.7:
            safety_penalty = 0.8  # 大幅降低
        elif min_distance < 1.3:
            safety_penalty = 0.5  # 中等降低
        else:
            safety_penalty = 0.0  # 无惩罚
        
        final_openness = base_openness * (1.0 - safety_penalty)
        return max(0.0, min(1.0, final_openness))

    def detect_multiple_paths_vector(self, ranges, angle_min, angle_increment):
        """
        基于高级扇区分析的多路径检测（使用potential_field方法的松耦合实现）
        """
        # 使用高级扇区分析方法
        sector_analysis = self.analyze_sectors_openness(ranges, angle_min, angle_increment)
        
        # 转换为简单的通路状态用于兼容现有接口
        paths_status = []
        sector_order = ['left', 'front', 'right']
        
        for direction in sector_order:
            if direction in sector_analysis:
                # 计算该主扇区的平均开放度
                openness_values = [sub['openness'] for sub in sector_analysis[direction]]
                avg_openness = np.mean(openness_values) if openness_values else 0.0
                max_openness = np.max(openness_values) if openness_values else 0.0
                
                # 通路判断：平均开放度>0.3 且 最大开放度>0.4
                is_clear = avg_openness > 0.3 and max_openness > 0.4
                paths_status.append(is_clear)
                
                self.get_logger().debug(
                    f"{direction.upper()} sector: avg_openness={avg_openness:.3f}, "
                    f"max_openness={max_openness:.3f}, clear={is_clear}"
                )
            else:
                paths_status.append(False)
        
        return paths_status

    def timer_callback(self):
        if len(self.lidar_ranges) == 0 or self.laser_data is None:
            return

        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        # === 使用统一的高级向量方法进行路径分析 ===
        # 使用向量方法检测多路径情况（这将成为主要的路径评估方法）
        paths_status = self.detect_multiple_paths_vector(ranges, angle_min, angle_increment)
        open_paths_count = sum(paths_status)
        
        # 将向量分析结果作为path_options使用，确保一致性
        path = [int(p) for p in paths_status]  # [左, 前, 右] 与向量分析结果一致
        
        # 检查前方障碍物（用于滑动窗口验证）
        immediate_front_blocked = False
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
            angle = angle_min + i * angle_increment
            # 检查前方±17度范围内是否有障碍物
            if abs(angle) < 0.3 and dist < self.min_obstacle_dist:
                immediate_front_blocked = True
                break
        
        # 多路径检测逻辑
        multipath_detected = False
        if open_paths_count >= 2:  # 至少有两条路径开放
            multipath_detected = True
            self.get_logger().info(f'Multiple paths detected! Open paths: Left={paths_status[0]}, Front={paths_status[1]}, Right={paths_status[2]}')
        
        # 综合判断是否需要阻塞路径 - 基于向量分析结果
        path_blocked = False
        block_reason = ""

        # 危险检测 - 降低敏感度，避免过度干扰
        all_dists = np.array(self.lidar_ranges)
        # 改为更严格的条件：0.4m距离且需要在前方较小角度范围内
        danger = False
        
        for i, dist in enumerate(all_dists):
            if not math.isfinite(dist):
                continue
            # 计算角度（简化计算，180度扫描）
            angle = (i / len(all_dists)) * 2 * math.pi - math.pi
            
            # 只有在前方较小范围内（±20度）且距离非常近（0.4m）才触发危险
            if abs(angle) < math.pi/9 and dist < 0.4:  # ±20度，0.4米
                danger = True
                self.get_logger().warn(f'Danger detected: obstacle at {dist:.2f}m in front')
                break

        # === 发布所有消息 ===
        # 发布统一的path_options（基于向量分析）
        path_msg = Int8MultiArray()
        path_msg.data = path
        self.pub.publish(path_msg)

        # 发布向量分析结果作为多路径检测（与path_options现在是相同的）
        multipath_msg = Int8MultiArray()
        multipath_msg.data = path  # 现在与path_options一致
        self.multipath_pub.publish(multipath_msg)

        # 发布路径阻塞状态
        blocked_msg = Bool()
        blocked_msg.data = path_blocked
        self.path_blocked_pub.publish(blocked_msg)

        # 发布危险状态
        danger_msg = Bool()
        danger_msg.data = bool(danger)
        self.danger_pub.publish(danger_msg)

        # 日志输出
        self.get_logger().info(f'/path_options: {path} | /multipath: {path} | /path_blocked: {path_blocked} ({block_reason}) | /danger_stop: {danger}')
        
def main(args=None):
    rclpy.init(args=args)
    node = PathEvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
