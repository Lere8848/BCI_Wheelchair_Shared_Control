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

    def is_front_blocked_with_sliding_window(self, current_detection):
        """
        使用滑动窗口机制判断前方是否真的被阻挡
        需要在窗口内连续检测到障碍物才认为真正被阻挡
        """
        # 将当前检测结果添加到历史记录
        self.obstacle_detection_history.append(current_detection)
        
        # 保持滑动窗口大小
        if len(self.obstacle_detection_history) > self.window_size:
            self.obstacle_detection_history.pop(0)
        
        # 检查最近的连续检测
        if len(self.obstacle_detection_history) < self.min_consecutive_detections:
            return False
        
        # 检查最后 min_consecutive_detections 次检测是否都为 True
        recent_detections = self.obstacle_detection_history[-self.min_consecutive_detections:]
        consecutive_true = all(recent_detections)
        
        if consecutive_true:
            self.get_logger().info(f'Confirmed obstacle: {self.min_consecutive_detections} consecutive detections')
        
        return consecutive_true

    def check_escape_space(self, ranges, angle_min, angle_increment):
        """
        检查轮椅周围是否有足够的转向空间，避免进入困境
        返回是否有足够的逃生空间
        """
        escape_distance = 1.0  # 需要的最小逃生距离(m)
        left_clear = True
        right_clear = True
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
            
            angle = angle_min + i * angle_increment
            
            # 检查左侧空间 (30-90度)
            if math.pi/6 < angle < math.pi/2 and dist < escape_distance:
                left_clear = False
            
            # 检查右侧空间 (-90到-30度)
            if -math.pi/2 < angle < -math.pi/6 and dist < escape_distance:
                right_clear = False
        
        # 如果左右都没有足够空间，说明可能陷入困境
        if not left_clear and not right_clear:
            self.get_logger().warn('Warning: Limited escape space detected on both sides!')
            return False
        
        return True

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
        # 左扇区：30°到90° (60度)
        # 前扇区：-30°到30° (60度)  
        # 右扇区：-90°到-30° (60度)
        
        sector_definitions = {
            0: (math.pi/6, math.pi/2),      # 左扇区：30°到90°
            1: (-math.pi/6, math.pi/6),     # 前扇区：-30°到30°
            2: (-math.pi/2, -math.pi/6)     # 右扇区：-90°到-30°
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

    def analyze_path_direction_vector(self, valid_points, direction_vector, direction_name):
        """
        分析特定方向向量上的通路状态
        返回该方向是否有清晰的通路
        """
        # 定义检测参数
        min_clear_distance = 1.2  # 最小清晰距离(m)
        cone_angle = math.pi/6  # 检测扇形角度(30度)
        
        # 计算方向向量的角度
        target_angle = math.atan2(direction_vector[1], direction_vector[0])
        
        # 在该方向的扇形区域内采样点
        points_in_direction = []
        distances_in_direction = []
        
        for point in valid_points:
            # 计算点相对于原点的角度
            point_angle = math.atan2(point[1], point[0])
            angle_diff = abs(point_angle - target_angle)
            
            # 处理角度跨越边界的情况
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # 如果点在目标方向的扇形范围内
            if angle_diff <= cone_angle:
                distance = np.linalg.norm(point)
                points_in_direction.append(point)
                distances_in_direction.append(distance)
        
        if len(points_in_direction) < 5:  # 需要足够的采样点
            return False
        
        distances_in_direction = np.array(distances_in_direction)
        
        # 分析距离分布
        near_obstacles = np.sum(distances_in_direction < 1.0)  # 1米内的障碍物
        far_clear_space = np.sum(distances_in_direction > min_clear_distance)  # 2米外的开放空间
        total_points = len(distances_in_direction)
        
        # 通路判断逻辑
        near_obstacle_ratio = near_obstacles / total_points
        far_clear_ratio = far_clear_space / total_points
        
        # 检查是否有清晰的通道边界
        has_clear_boundary = self.detect_corridor_boundary_vector(
            points_in_direction, direction_vector
        )
        
        # 通路开放条件：
        # 1. 近距离障碍物比例不能太高（避免前方直接被堵）
        # 2. 远距离必须有足够的开放空间
        # 3. 必须检测到明确的通道边界结构
        path_clear = (
            near_obstacle_ratio < 0.3 and  # 近距离阻塞率小于30%
            far_clear_ratio > 0.4 and      # 远距离开放率大于40%
            has_clear_boundary              # 有明确的通道边界
        )
        
        self.get_logger().debug(
            f'{direction_name} direction analysis: '
            f'Near obstacles: {near_obstacle_ratio:.2f}, '
            f'Far clear: {far_clear_ratio:.2f}, '
            f'Boundary: {has_clear_boundary}, '
            f'Result: {path_clear}'
        )
        
        return path_clear

    def detect_corridor_boundary_vector(self, points_in_direction, direction_vector):
        """
        使用向量方法检测通道边界
        通过分析点云的梯度变化来识别墙壁和开放空间的边界
        """
        if len(points_in_direction) < 5:
            return False
        
        points = np.array(points_in_direction)
        
        # 将点投影到垂直于检测方向的线上（检测左右边界）
        perpendicular_vector = np.array([-direction_vector[1], direction_vector[0]])
        
        # 计算每个点在垂直方向上的投影
        lateral_positions = []
        distances = []
        
        for point in points:
            # 在检测方向上的距离
            distance = np.dot(point, direction_vector)
            # 在垂直方向上的位置（左右偏移）
            lateral_pos = np.dot(point, perpendicular_vector)
            
            lateral_positions.append(lateral_pos)
            distances.append(distance)
        
        lateral_positions = np.array(lateral_positions)
        distances = np.array(distances)
        
        # 检查是否有明显的左右边界
        # 如果存在通道，应该能在左右两侧检测到障碍物聚集
        left_boundary_points = np.sum(lateral_positions > 0.5)   # 左侧边界（正Y方向）
        right_boundary_points = np.sum(lateral_positions < -0.5)  # 右侧边界（负Y方向）
        
        # 检查距离的梯度变化
        if len(distances) > 3:
            # 按照横向位置排序
            sorted_indices = np.argsort(lateral_positions)
            sorted_distances = distances[sorted_indices]
            
            # 计算距离梯度（相邻点距离差）
            distance_gradients = np.diff(sorted_distances)
            large_gradients = np.sum(np.abs(distance_gradients) > 0.5)
            
            # 如果有明显的距离跳跃，说明存在结构边界
            has_distance_boundary = large_gradients > 0
        else:
            has_distance_boundary = False
        
        # 边界检测条件：
        # 1. 左右两侧都有一定数量的边界点，或
        # 2. 存在明显的距离梯度变化
        has_boundary = (
            (left_boundary_points > 2 and right_boundary_points > 2) or
            has_distance_boundary
        )
        
        return has_boundary

    def check_wall_collision_risk(self, ranges, angle_min, angle_increment):
        """
        改进的墙面碰撞风险检测
        检查更小的角度范围和更近的距离
        """
        collision_risk = False
        min_wall_distance = float('inf')
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
                
            angle = angle_min + i * angle_increment
            
            # 检查正前方更小的角度范围 (±10度)
            if abs(angle) < 0.175:  # 约±10度
                if dist < min_wall_distance:
                    min_wall_distance = dist
                if dist < self.wall_detection_distance:
                    collision_risk = True
                    self.get_logger().warn(f'Wall collision risk detected! Distance: {dist:.2f}m at angle: {math.degrees(angle):.1f}°')
        
        return collision_risk, min_wall_distance

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
        
        # 使用滑动窗口机制确认前方是否真正被阻挡
        front_blocked = self.is_front_blocked_with_sliding_window(immediate_front_blocked)
        
        # 检查逃生空间
        has_escape_space = self.check_escape_space(ranges, angle_min, angle_increment)
        
        # 检查墙面碰撞风险
        wall_collision_risk, min_wall_distance = self.check_wall_collision_risk(ranges, angle_min, angle_increment)
        
        # 多路径检测逻辑
        multipath_detected = False
        if open_paths_count >= 2:  # 至少有两条路径开放
            multipath_detected = True
            self.get_logger().info(f'Multiple paths detected! Open paths: Left={paths_status[0]}, Front={paths_status[1]}, Right={paths_status[2]}')
        
        # 综合判断是否需要阻塞路径 - 基于向量分析结果
        path_blocked = False
        block_reason = ""
        
        if wall_collision_risk:
            path_blocked = True
            block_reason = "Wall collision risk"
        elif front_blocked and not has_escape_space:
            path_blocked = True  
            block_reason = "Front blocked with limited escape space"
        elif multipath_detected:
            path_blocked = True
            block_reason = "Multiple paths detected - awaiting user selection"
        elif not paths_status[1]:  # 如果向量分析显示前方不通畅
            path_blocked = True
            block_reason = "Front path blocked by vector analysis"

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
        
        if path_blocked:
            self.get_logger().debug(f'Path analysis details:')
            self.get_logger().debug(f'  Wall collision: {wall_collision_risk}, distance: {min_wall_distance:.2f}m')
            self.get_logger().debug(f'  Front blocked: {front_blocked}, escape space: {has_escape_space}')
            self.get_logger().debug(f'  Multipath detected: {multipath_detected}, open paths: {open_paths_count}')

def main(args=None):
    rclpy.init(args=args)
    node = PathEvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
