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

    def detect_multiple_paths_vector(self, ranges, angle_min, angle_increment):
        """
        使用向量方法检测多路径
        通过分析激光点云的向量分布来识别通道和路口
        """
        # 将极坐标转换为笛卡尔坐标系下的向量
        valid_points = []
        valid_angles = []
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist) or dist > 5.0:  # 限制最大检测距离
                continue
                
            angle = angle_min + i * angle_increment
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            
            valid_points.append([x, y])
            valid_angles.append(angle)
        
        if len(valid_points) < 10:  # 需要足够的点进行分析
            return [False, True, False]
        
        valid_points = np.array(valid_points)
        
        # 定义三个60度扇区划分（与potential_field_planner统一）
        # 前方180度范围，每个方向各60度
        # 注意：根据Unity雷达数据实际测试，需要调整左右定义
        # 左扇区：-90°到-30° (实际对应Unity中的左侧)
        # 前扇区：-30°到30° (60度)  
        # 右扇区：30°到90° (实际对应Unity中的右侧)
        
        sector_definitions = {
            0: (-math.pi/2, -math.pi/6),    # 左扇区：-90°到-30° (调整后)
            1: (-math.pi/6, math.pi/6),     # 前扇区：-30°到30°
            2: (math.pi/6, math.pi/2)       # 右扇区：30°到90° (调整后)
        }
        
        sector_names = ["LEFT", "FRONT", "RIGHT"]
        paths_status = [False, False, False]
        
        for sector_idx, (start_angle, end_angle) in sector_definitions.items():
            # 分析该扇区的通路状态
            path_clear = self.analyze_sector_openness(
                valid_points, start_angle, end_angle, sector_names[sector_idx]
            )
            paths_status[sector_idx] = path_clear
        
        return paths_status

    def analyze_sector_openness(self, valid_points, start_angle, end_angle, sector_name):
        """
        分析特定扇区的开放程度
        参数:
            valid_points: 有效的激光点阵列
            start_angle: 扇区起始角度
            end_angle: 扇区结束角度  
            sector_name: 扇区名称（用于日志）
        返回:
            bool: 该扇区是否通畅
        """
        if len(valid_points) == 0:
            return False
            
        # 将极坐标转换为角度
        angles = np.arctan2(valid_points[:, 1], valid_points[:, 0])
        
        # 处理角度跨越问题（如-90°到-30°）
        if start_angle > end_angle:
            # 跨越0度的情况，例如从330°到30°
            sector_mask = (angles >= start_angle) | (angles <= end_angle)
        else:
            # 正常情况
            sector_mask = (angles >= start_angle) & (angles <= end_angle)
        
        sector_points = valid_points[sector_mask]
        
        if len(sector_points) == 0:
            self.get_logger().debug(f"{sector_name} sector: No points in range")
            return True  # 没有障碍物点，认为通畅
        
        # 计算该扇区内的最近距离
        distances = np.linalg.norm(sector_points, axis=1)
        min_distance = np.min(distances)
        avg_distance = np.mean(distances)
        
        # 判断通畅标准：最近距离>1.2m 且 平均距离>1.5m
        is_clear = min_distance > 1.2 and avg_distance > 1.5
        
        self.get_logger().debug(
            f"{sector_name} sector: min_dist={min_distance:.2f}m, "
            f"avg_dist={avg_distance:.2f}m, points={len(sector_points)}, "
            f"clear={is_clear}"
        )
        
        return is_clear

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
            # 计算角度（简化计算，假设360度扫描）
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
