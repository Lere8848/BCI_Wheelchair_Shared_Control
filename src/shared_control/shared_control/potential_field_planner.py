# potential_field_planner.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import numpy as np
import math

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        # 订阅激光雷达数据
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan',
            self.laser_callback, 
            10
        )
        # 订阅用户意图（0=左, 1=前, 2=右）
        from std_msgs.msg import Int8
        self.user_dir_sub = self.create_subscription(
            Int8,
            '/user_direction',
            self.user_dir_callback,
            10
        )
        # 发布计算出的路径指令
        self.path_pub = self.create_publisher(
            Twist, 
            '/auto_cmd_vel', 
            10
        )
        # 发布是否有障碍物阻挡前进的信号
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/path_blocked',
            10
        )
        
        # 发布多路径检测信号 (用于触发用户选择)
        from std_msgs.msg import Int8MultiArray
        self.multipath_pub = self.create_publisher(
            Int8MultiArray,
            '/multipath_detected',
            10
        )
        
        # 发布轮椅意图给Unity端
        from std_msgs.msg import String
        self.wheelchair_intent_pub = self.create_publisher(
            String,
            '/wheelchair_intent',
            10
        )

        # ====== parameters =====
        self.goal_dist = 3.0  # 吸引点距离(m)
        self.obstacle_influence = 0.8  # 障碍物影响范围(m)
        self.repulsive_coef = 0.2  # 排斥力系数 (降低，使避障更温和)
        self.attractive_coef = 0.1  # 吸引力系数
        self.min_obstacle_dist = 0.8  # 最小安全距离(m) (增加，提前检测障碍)
        
        # 多路径检测参数
        self.path_detection_distance = 2.0  # 多路径检测距离(m)
        self.path_width_threshold = 1.0     # 路径宽度阈值(m)
        self.wall_detection_distance = 0.5  # 墙面检测距离(m)
        
        # 滑动窗口障碍物检测参数
        self.window_size = 5  # 滑动窗口大小
        self.min_consecutive_detections = 2  # 连续检测到障碍物的最小次数
        self.obstacle_detection_history = []  # 障碍物检测历史记录
        
        # 用户意图，默认前进
        self.user_direction = 1  # 0=左, 1=前, 2=右
        
        # 连续意图生成参数
        self.continuous_intent_enabled = True  # 启用连续意图
        self.num_sectors = 5  # 将前方分为5个扇形区域
        self.sector_angle = math.pi / 3  # 总扇形角度(60度)
        self.continuous_target_angle = 0.0  # 当前连续目标角度
        self.intent_smoothing_factor = 0.3  # 意图平滑系数
        
        # 路径计算定时器
        self.timer = self.create_timer(0.1, self.calculate_path)
        # 激光数据存储
        self.laser_data = None
        self.get_logger().info('Potential Field Planner Node Initialized (Continuous Intent with Vector Analysis)')

    def user_dir_callback(self, msg):
        # 0=左, 1=前, 2=右
        if msg.data in [0, 1, 2]:
            self.user_direction = msg.data
            self.get_logger().info(f'Received user direction: {self.user_direction}')
        else:
            self.get_logger().warn(f'Invalid user direction: {msg.data}')
    
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
        
        # 定义三个检测区域的向量方向
        left_direction = np.array([math.cos(math.pi/4), math.sin(math.pi/4)])    # 45度方向
        front_direction = np.array([1.0, 0.0])                                   # 0度方向  
        right_direction = np.array([math.cos(-math.pi/4), math.sin(-math.pi/4)]) # -45度方向
        
        detection_vectors = [left_direction, front_direction, right_direction]
        detection_names = ["LEFT", "FRONT", "RIGHT"]
        
        paths_status = [False, False, False]
        
        for dir_idx, detection_vec in enumerate(detection_vectors):
            # 计算该方向的通路状态
            path_clear = self.analyze_path_direction_vector(
                valid_points, detection_vec, detection_names[dir_idx]
            )
            paths_status[dir_idx] = path_clear
        
        return paths_status

    def analyze_path_direction_vector(self, points, direction_vector, direction_name):
        """
        分析特定方向向量上的通路状态
        返回该方向是否有清晰的通路
        """
        # 定义检测参数
        # corridor_width = 1.5  # 通道宽度(m)
        min_clear_distance = 1.2  # 最小清晰距离(m)
        cone_angle = math.pi/6  # 检测扇形角度(30度)
        
        # 计算方向向量的角度
        target_angle = math.atan2(direction_vector[1], direction_vector[0])
        
        # 在该方向的扇形区域内采样点
        points_in_direction = []
        distances_in_direction = []
        
        for point in points:
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
        left_boundary_points = np.sum(lateral_positions < -0.5)  # 左侧边界
        right_boundary_points = np.sum(lateral_positions > 0.5)   # 右侧边界
        
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

    def analyze_continuous_intent(self, ranges, angle_min, angle_increment):
        """
        连续意图分析：将前方区域细分为多个扇区，计算最优的连续移动方向
        返回连续的目标角度（弧度）
        """
        if not self.continuous_intent_enabled:
            # 如果未启用连续意图，返回离散方向对应的角度
            angle_map = {0: math.pi/4, 1: 0.0, 2: -math.pi/4}
            return angle_map.get(self.user_direction, 0.0)
        
        # 将前方60度区域分为5个扇区，每个扇区12度
        sector_angles = []
        sector_openness = []
        
        # 计算每个扇区的中心角度
        start_angle = -self.sector_angle / 2  # -30度
        sector_width = self.sector_angle / self.num_sectors  # 12度
        
        for i in range(self.num_sectors):
            sector_center = start_angle + (i + 0.5) * sector_width
            sector_angles.append(sector_center)
            
            # 分析该扇区的开放程度
            openness = self.calculate_sector_openness(
                ranges, angle_min, angle_increment, 
                sector_center, sector_width
            )
            sector_openness.append(openness)
        
        # 记录扇区分析结果
        self.get_logger().debug(
            f'Sector analysis: '
            f'Angles: {[math.degrees(a) for a in sector_angles]}, '
            f'Openness: {[f"{o:.2f}" for o in sector_openness]}'
        )
        
        # 基于用户意图、环境开放程度和避障需求计算最优角度
        optimal_angle = self.calculate_optimal_direction(
            sector_angles, sector_openness, ranges, angle_min, angle_increment
        )
        
        # 如果没有安全的移动方向，返回None表示应该停止
        if optimal_angle is None:
            self.get_logger().error('No safe direction found - emergency stop required')
            return None
        
        # 可视化分析结果（每5次输出一次，避免日志过多）
        if hasattr(self, '_viz_counter'):
            self._viz_counter += 1
        else:
            self._viz_counter = 0
            
        if self._viz_counter % 50 == 0:  # 每5秒输出一次可视化结果
            self.visualize_continuous_intent(sector_angles, sector_openness)
        
        # 应用平滑滤波，避免突然的方向变化
        self.continuous_target_angle = (
            self.intent_smoothing_factor * optimal_angle + 
            (1 - self.intent_smoothing_factor) * self.continuous_target_angle
        )
        
        self.get_logger().debug(
            f'Continuous intent: Raw optimal={math.degrees(optimal_angle):.1f}°, '
            f'Smoothed target={math.degrees(self.continuous_target_angle):.1f}°'
        )
        
        return self.continuous_target_angle

    def calculate_sector_openness(self, ranges, angle_min, angle_increment, 
                                 sector_center, sector_width):
        """
        计算特定扇区的开放程度
        返回0-1之间的开放度数值，1表示完全开放，0表示完全阻塞
        """
        sector_min = sector_center - sector_width / 2
        sector_max = sector_center + sector_width / 2
        
        distances_in_sector = []
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
                
            angle = angle_min + i * angle_increment
            
            # 检查是否在当前扇区范围内
            if sector_min <= angle <= sector_max:
                distances_in_sector.append(dist)
        
        if len(distances_in_sector) == 0:
            return 0.5  # 无数据时返回中等开放度
        
        distances = np.array(distances_in_sector)
        
        # 开放度计算方法：
        # 1. 平均距离权重
        avg_distance = np.mean(distances)
        distance_score = min(1.0, avg_distance / 3.0)  # 3米为满分距离
        
        # 2. 最小距离安全性
        min_distance = np.min(distances)
        safety_score = min(1.0, min_distance / 1.0)  # 1米为安全距离
        
        # 3. 距离一致性（避免障碍物过于密集的区域）
        distance_std = np.std(distances)
        consistency_score = max(0.0, 1.0 - distance_std / 1.0)  # 标准差小于1米为好

        # 综合开放度评分
        openness = (
            0.4 * distance_score +     # 40%权重给平均距离
            0.4 * safety_score +       # 40%权重给最小安全距离  
            0.2 * consistency_score    # 20%权重给一致性
        )
        
        return max(0.0, min(1.0, openness))

    def calculate_optimal_direction(self, sector_angles, sector_openness, ranges, angle_min, angle_increment):
        """
        基于扇区开放度、用户意图和避障需求计算最优移动方向
        整合势场避障机制，实现连续的避障意图
        """
        # 权重配置
        user_preference_weight = 0.4    # 用户意图权重（降低，给避障更多权重）
        environment_weight = 0.2        # 环境开放度权重
        avoidance_weight = 0.4          # 避障紧急度权重

        # 将用户意图转换为偏好角度
        user_angle_map = {0: math.pi/4, 1: 0.0, 2: -math.pi/4}
        preferred_angle = user_angle_map.get(self.user_direction, 0.0)
        
        # 检测紧急避障需求
        urgent_avoidance_angle = self.detect_urgent_avoidance_direction(
            ranges, angle_min, angle_increment
        )
        
        # 计算每个扇区的综合评分
        sector_scores = []
        
        for i, (angle, openness) in enumerate(zip(sector_angles, sector_openness)):
            # 1. 用户偏好评分
            angle_diff = abs(angle - preferred_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            user_score = math.exp(-2 * angle_diff)
            
            # 2. 环境开放度评分
            env_score = openness
            
            # 3. 避障紧急度评分
            avoidance_score = 1.0  # 默认无需避障
            if urgent_avoidance_angle is not None:
                # 如果检测到紧急避障需求，评估该扇区是否是好的避障方向
                avoidance_diff = abs(angle - urgent_avoidance_angle)
                if avoidance_diff > math.pi:
                    avoidance_diff = 2 * math.pi - avoidance_diff
                avoidance_score = math.exp(-3 * avoidance_diff)  # 更陡峭的衰减
                
                self.get_logger().debug(
                    f'Urgent avoidance detected: target={math.degrees(urgent_avoidance_angle):.1f}°, '
                    f'sector{i}={math.degrees(angle):.1f}°, avoidance_score={avoidance_score:.2f}'
                )
            
            # 综合评分
            total_score = (
                user_preference_weight * user_score + 
                environment_weight * env_score +
                avoidance_weight * avoidance_score
            )
            
            sector_scores.append(total_score)
            
            self.get_logger().debug(
                f'Sector {i}: angle={math.degrees(angle):.1f}°, '
                f'openness={openness:.2f}, user={user_score:.2f}, '
                f'avoid={avoidance_score:.2f}, total={total_score:.2f}'
            )
        
        # 找到评分最高的扇区
        best_sector_idx = np.argmax(sector_scores)
        optimal_angle = sector_angles[best_sector_idx]
        
        # 安全性检查：如果最优扇区仍然不够安全，强制选择最安全的扇区
        if sector_openness[best_sector_idx] < 0.2:
            self.get_logger().warn(
                f'Best sector unsafe (openness={sector_openness[best_sector_idx]:.2f}), '
                f'forcing safety override'
            )
            
            # 强制选择开放度最高的扇区
            safest_sector_idx = np.argmax(sector_openness)
            if sector_openness[safest_sector_idx] > 0.4:
                optimal_angle = sector_angles[safest_sector_idx]
                self.get_logger().info(
                    f'Safety override: selected sector {safest_sector_idx} '
                    f'with angle {math.degrees(optimal_angle):.1f}°'
                )
            else:
                # 如果没有安全扇区，返回None表示应该停止
                self.get_logger().error('No safe sectors available! Should stop.')
                return None
        
        return optimal_angle

    def detect_urgent_avoidance_direction(self, ranges, angle_min, angle_increment):
        """
        检测是否需要紧急避障，并计算避障方向
        返回建议的避障角度，如果不需要避障则返回None
        """
        danger_distance = 1.0  # 危险距离阈值
        critical_distance = 0.6  # 极度危险距离
        
        # 检测前方危险区域
        front_dangers = []
        critical_dangers = []
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
                
            angle = angle_min + i * angle_increment
            
            # 只关注前方120度范围内的障碍物
            if abs(angle) <= math.pi/3:  # ±60度
                if dist < critical_distance:
                    critical_dangers.append((angle, dist))
                elif dist < danger_distance:
                    front_dangers.append((angle, dist))
        
        # 如果有极度危险的障碍物，立即计算避障方向
        if critical_dangers:
            self.get_logger().warn(f'Critical danger detected: {len(critical_dangers)} obstacles within {critical_distance}m')
            return self.calculate_emergency_avoidance_angle(critical_dangers, ranges, angle_min, angle_increment)
        
        # 如果有一般危险，计算预防性避障方向
        if len(front_dangers) >= 3:  # 多个障碍物表示需要避障
            self.get_logger().info(f'Multiple front obstacles detected: {len(front_dangers)} within {danger_distance}m')
            return self.calculate_preventive_avoidance_angle(front_dangers, ranges, angle_min, angle_increment)
        
        return None  # 无需避障

    def calculate_emergency_avoidance_angle(self, critical_dangers, ranges, angle_min, angle_increment):
        """
        计算紧急避障角度
        """
        # 找到危险障碍物的角度范围
        danger_angles = [angle for angle, _ in critical_dangers]
        min_danger_angle = min(danger_angles)
        max_danger_angle = max(danger_angles)
        
        # 检查左右两侧的开放空间
        left_space = self.check_lateral_space(ranges, angle_min, angle_increment, 
                                            math.pi/6, math.pi/2)  # 30-90度
        right_space = self.check_lateral_space(ranges, angle_min, angle_increment, 
                                             -math.pi/2, -math.pi/6)  # -90到-30度
        
        # 选择更开放的一侧
        if left_space > right_space and left_space > 1.5:
            avoidance_angle = math.pi/3  # 60度向左
            self.get_logger().info(f'Emergency avoidance: LEFT (space={left_space:.2f}m)')
        elif right_space > 1.5:
            avoidance_angle = -math.pi/3  # 60度向右
            self.get_logger().info(f'Emergency avoidance: RIGHT (space={right_space:.2f}m)')
        else:
            # 两侧都不安全，选择相对安全的一侧
            if left_space >= right_space:
                avoidance_angle = math.pi/4  # 45度向左
                self.get_logger().warn(f'Limited emergency avoidance: LEFT (space={left_space:.2f}m)')
            else:
                avoidance_angle = -math.pi/4  # 45度向右
                self.get_logger().warn(f'Limited emergency avoidance: RIGHT (space={right_space:.2f}m)')
        
        return avoidance_angle

    def calculate_preventive_avoidance_angle(self, front_dangers, ranges, angle_min, angle_increment):
        """
        计算预防性避障角度
        """
        # 计算障碍物的重心位置
        obstacle_center_angle = np.mean([angle for angle, _ in front_dangers])
        
        # 检查左右空间
        left_space = self.check_lateral_space(ranges, angle_min, angle_increment, 
                                            math.pi/6, math.pi/2)
        right_space = self.check_lateral_space(ranges, angle_min, angle_increment, 
                                             -math.pi/2, -math.pi/6)
        
        # 选择远离障碍物重心且空间较大的方向
        if obstacle_center_angle > 0:  # 障碍物偏右，优先向左避障
            if left_space > 1.0:
                avoidance_angle = math.pi/4  # 45度向左
                self.get_logger().info(f'Preventive avoidance: LEFT away from right obstacles')
            elif right_space > left_space:
                avoidance_angle = -math.pi/6  # 30度向右
                self.get_logger().info(f'Preventive avoidance: slight RIGHT (limited left space)')
            else:
                avoidance_angle = math.pi/6  # 30度向左
                self.get_logger().info(f'Preventive avoidance: slight LEFT (limited space)')
        else:  # 障碍物偏左，优先向右避障
            if right_space > 1.0:
                avoidance_angle = -math.pi/4  # 45度向右
                self.get_logger().info(f'Preventive avoidance: RIGHT away from left obstacles')
            elif left_space > right_space:
                avoidance_angle = math.pi/6  # 30度向左
                self.get_logger().info(f'Preventive avoidance: slight LEFT (limited right space)')
            else:
                avoidance_angle = -math.pi/6  # 30度向右
                self.get_logger().info(f'Preventive avoidance: slight RIGHT (limited space)')
        
        return avoidance_angle

    def check_lateral_space(self, ranges, angle_min, angle_increment, start_angle, end_angle):
        """
        检查指定角度范围内的平均距离（空间大小）
        """
        distances = []
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
                
            angle = angle_min + i * angle_increment
            
            if start_angle <= angle <= end_angle:
                distances.append(dist)
        
        if distances:
            return np.mean(distances)
        else:
            return 0.0

    def visualize_continuous_intent(self, sector_angles, sector_openness):
        """
        可视化连续意图分析结果（通过日志输出）
        """
        if not sector_angles or not sector_openness:
            return
        
        self.get_logger().info("=== 连续意图分析结果 ===")
        
        # 创建扇区状态的可视化表示
        visual_map = []
        for i, (angle, openness) in enumerate(zip(sector_angles, sector_openness)):
            angle_deg = math.degrees(angle)
            
            # 根据开放度生成视觉指示符
            if openness > 0.7:
                status = "🟢"  # 绿色：非常开放
            elif openness > 0.4:
                status = "🟡"  # 黄色：部分开放
            elif openness > 0.2:
                status = "🟠"  # 橙色：轻微阻塞
            else:
                status = "🔴"  # 红色：严重阻塞
            
            visual_map.append(f"{status}({angle_deg:+5.1f}°:{openness:.2f})")
        
        self.get_logger().info(f"扇区状态: {' '.join(visual_map)}")
        
        # 显示用户意图
        direction_names = ["LEFT", "FORWARD", "RIGHT"]
        user_intent = direction_names[self.user_direction]
        self.get_logger().info(f"用户意图: {user_intent}")
        
        # 显示最终选择的连续目标角度
        target_deg = math.degrees(self.continuous_target_angle)
        self.get_logger().info(f"连续目标方向: {target_deg:+6.1f}°")
        self.get_logger().info("====================")

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
    
    def laser_callback(self, msg):
        self.laser_data = msg
        # self.get_logger().debug('收到激光扫描数据')
    
    def calculate_path(self):
        if self.laser_data is None:
            self.get_logger().warn('Laser data not received yet')
            return

        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        # 使用连续意图分析计算最优移动方向
        continuous_goal_angle = self.analyze_continuous_intent(
            ranges, angle_min, angle_increment
        )

        # 检查是否需要紧急停止（连续意图分析返回None）
        emergency_stop_due_to_safety = (continuous_goal_angle is None)
        
        if emergency_stop_due_to_safety:
            self.get_logger().error('Emergency stop triggered by safety analysis')
            continuous_goal_angle = 0.0  # 设置默认值以避免后续计算错误

        # 基于连续意图设置吸引力方向
        goal = Point()
        goal.x = self.goal_dist * math.cos(continuous_goal_angle)
        goal.y = self.goal_dist * math.sin(continuous_goal_angle)

        # 计算吸引力 (指向连续计算的目标点)
        att_force_x = self.attractive_coef * goal.x
        att_force_y = self.attractive_coef * goal.y

        # 计算激光数据中的排斥力
        rep_force_x = 0.0
        rep_force_y = 0.0

        # 记录最小距离和初步障碍物检测
        min_dist = float('inf')
        immediate_front_blocked = False

        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
            if dist < min_dist:
                min_dist = dist
            angle = angle_min + i * angle_increment
            if dist < self.obstacle_influence:
                magnitude = self.repulsive_coef * (1.0 / dist - 1.0 / self.obstacle_influence)
                obstacle_x = dist * math.cos(angle)
                obstacle_y = dist * math.sin(angle)
                if obstacle_x == 0 and obstacle_y == 0:
                    continue
                norm = math.sqrt(obstacle_x**2 + obstacle_y**2)
                obstacle_x /= norm
                obstacle_y /= norm
                rep_force_x += magnitude * (-obstacle_x)
                rep_force_y += magnitude * (-obstacle_y)
            # 检查前方是否有立即的障碍物（约±17度范围内）
            if abs(angle) < 0.3 and dist < self.min_obstacle_dist:
                immediate_front_blocked = True

        # 检查逃生空间
        has_escape_space = self.check_escape_space(ranges, angle_min, angle_increment)
        
        # 使用向量方法检测多路径情况
        paths_status = self.detect_multiple_paths_vector(ranges, angle_min, angle_increment)
        open_paths_count = sum(paths_status)
        
        # 检查墙面碰撞风险
        wall_collision_risk, min_wall_distance = self.check_wall_collision_risk(ranges, angle_min, angle_increment)
        
        # 使用滑动窗口机制确认前方是否真正被阻挡
        front_blocked = self.is_front_blocked_with_sliding_window(immediate_front_blocked)
        
        # 多路径检测逻辑
        multipath_detected = False
        if open_paths_count >= 2:  # 至少有两条路径开放
            multipath_detected = True
            self.get_logger().info(f'Multiple paths detected! Open paths: Left={paths_status[0]}, Front={paths_status[1]}, Right={paths_status[2]}')
        
        # 发布多路径检测结果
        from std_msgs.msg import Int8MultiArray
        multipath_msg = Int8MultiArray()
        multipath_msg.data = [int(p) for p in paths_status]  # 转换为int列表
        self.multipath_pub.publish(multipath_msg)
        
        # 综合判断是否需要停止
        should_stop = False
        stop_reason = ""
        wheelchair_intent = ""  # 轮椅当前意图
        
        if emergency_stop_due_to_safety:
            should_stop = True
            stop_reason = "Safety analysis indicates no safe direction available"
            wheelchair_intent = "EMERGENCY_STOP: No safe movement direction found"
        elif wall_collision_risk:
            should_stop = True
            stop_reason = "Wall collision risk"
            wheelchair_intent = "EMERGENCY_STOP: Wall collision risk detected"
        elif front_blocked and not has_escape_space:
            should_stop = True  
            stop_reason = "Front blocked with limited escape space"
            wheelchair_intent = "EMERGENCY_STOP: Trapped situation detected"
        elif multipath_detected:
            should_stop = True
            stop_reason = "Multiple paths detected - awaiting user selection"
            wheelchair_intent = f"WAITING_FOR_USER: Multiple paths available - Left:{paths_status[0]} Front:{paths_status[1]} Right:{paths_status[2]}"
        elif front_blocked:
            should_stop = True
            stop_reason = "Front path blocked"
            wheelchair_intent = "OBSTACLE_AVOIDANCE: Front path blocked"
        else:
            # 正常移动状态 - 包含连续意图和避障信息
            direction_names = ["LEFT", "FORWARD", "RIGHT"]
            user_intent_name = direction_names[self.user_direction]
            
            # 将连续目标角度转换为度数
            target_angle_deg = math.degrees(continuous_goal_angle)
            
            # 检查是否正在进行避障
            if abs(continuous_goal_angle) > math.pi/6:  # 偏离正前方超过30度
                wheelchair_intent = (
                    f"CONTINUOUS_AVOIDING: User intent={user_intent_name}, "
                    f"Avoidance direction={target_angle_deg:.1f}°, "
                    f"Active obstacle avoidance"
                )
            else:
                wheelchair_intent = (
                    f"CONTINUOUS_MOVING: User intent={user_intent_name}, "
                    f"Computed direction={target_angle_deg:.1f}°, "
                    f"Normal navigation"
                )
        
        # 发布轮椅意图给Unity端
        from std_msgs.msg import String
        intent_msg = String()
        intent_msg.data = wheelchair_intent
        self.wheelchair_intent_pub.publish(intent_msg)
        
        # 如果前方被阻挡且缺乏逃生空间，提前警告
        if front_blocked and not has_escape_space:
            self.get_logger().warn('DANGER: Front blocked and limited escape space!')

        # 发布前方是否被阻挡的信息
        blocked_msg = Bool()
        blocked_msg.data = should_stop
        self.obstacle_pub.publish(blocked_msg)

        # 合并吸引力和排斥力
        total_force_x = att_force_x + rep_force_x
        total_force_y = att_force_y + rep_force_y

        # 转换为线速度和角速度
        cmd = Twist()
        if should_stop:
            cmd.linear.x = 0.0
            self.get_logger().warn(f'Stopping: {stop_reason}')
        else:
            cmd.linear.x = min(0.5, total_force_x)
        cmd.angular.z = min(0.5, max(-0.5, total_force_y))
        
        # 记录详细日志
        self.get_logger().debug(f'Min distance: {min_dist:.2f}m, Wall distance: {min_wall_distance:.2f}m')
        self.get_logger().debug(f'Immediate blocked: {immediate_front_blocked}, Confirmed blocked: {front_blocked}')
        self.get_logger().debug(f'Paths open: L={paths_status[0]}, F={paths_status[1]}, R={paths_status[2]} (Count: {open_paths_count})')
        self.get_logger().debug(f'Escape space: {has_escape_space}, Wall risk: {wall_collision_risk}')
        self.get_logger().debug(f'Emergency safety stop: {emergency_stop_due_to_safety}')
        self.get_logger().debug(f'Continuous target angle: {math.degrees(continuous_goal_angle):.1f}°')
        self.get_logger().debug(f'Should stop: {should_stop} ({stop_reason})')
        self.get_logger().debug(f'Wheelchair intent: {wheelchair_intent}')
        self.get_logger().debug(f'Total force: ({total_force_x:.2f}, {total_force_y:.2f})')
        self.get_logger().debug(f'Command: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f})')
        
        # 发布移动指令
        self.path_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
