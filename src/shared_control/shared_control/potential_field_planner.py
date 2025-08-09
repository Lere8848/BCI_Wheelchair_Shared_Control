# potential_field_planner.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Int8, Int8MultiArray, String
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
        self.user_dir_sub = self.create_subscription(
            Int8,
            '/user_direction',
            self.user_dir_callback,
            10
        )
        
        # 订阅路径评估结果
        self.path_blocked_sub = self.create_subscription(
            Bool,
            '/path_blocked',
            self.path_blocked_callback,
            10
        )
        self.multipath_sub = self.create_subscription(
            Int8MultiArray,
            '/multipath_detected',
            self.multipath_callback,
            10
        )
        
        # 发布计算出的路径指令
        self.path_pub = self.create_publisher(
            Twist, 
            '/auto_cmd_vel', 
            10
        )
        
        # 发布轮椅意图给Unity端
        self.wheelchair_intent_pub = self.create_publisher(
            String,
            '/wheelchair_intent',
            10
        )

        # ====== parameters =====
        self.goal_dist = 3.0  # 吸引点距离(m)
        self.obstacle_influence = 1.2  # 障碍物影响范围(m) - 增加影响范围
        self.repulsive_coef = 0.5  # 排斥力系数 - 增强排斥力
        self.attractive_coef = 0.4  # 吸引力系数 - 增强吸引力
        
        # 用户意图，默认前进
        self.user_direction = 1  # 0=左, 1=前, 2=右
        
        # 连续意图生成参数
        self.continuous_intent_enabled = True  # 启用连续意图
        self.num_sectors = 5  # 将前方分为5个扇形区域
        self.sector_angle = math.pi / 3  # 总扇形角度(60度)
        self.continuous_target_angle = 0.0  # 当前连续目标角度
        self.intent_smoothing_factor = 0.3  # 意图平滑系数
        
        # 来自路径评估节点的状态
        self.path_blocked = False
        self.multipath_detected = False
        self.multipath_status = [False, False, False]
        
        # 路径计算定时器
        self.timer = self.create_timer(0.1, self.calculate_path)
        # 激光数据存储
        self.laser_data = None
        self.get_logger().info('Potential Field Planner Node Initialized (Focuses on Force Calculation)')

    def user_dir_callback(self, msg):
        # 0=左, 1=前, 2=右
        if msg.data in [0, 1, 2]:
            self.user_direction = msg.data
            self.get_logger().info(f'Received user direction: {self.user_direction}')
        else:
            self.get_logger().warn(f'Invalid user direction: {msg.data}')
    
    def path_blocked_callback(self, msg):
        """接收来自路径评估节点的阻塞状态"""
        self.path_blocked = msg.data
        if self.path_blocked:
            self.get_logger().debug('Path blocked signal received from PathEvalNode')
    
    def multipath_callback(self, msg):
        """接收来自路径评估节点的多路径检测结果"""
        if len(msg.data) == 3:
            self.multipath_status = [bool(x) for x in msg.data]
            self.multipath_detected = sum(self.multipath_status) >= 2
            if self.multipath_detected:
                self.get_logger().debug(f'Multipath detected: L={self.multipath_status[0]}, F={self.multipath_status[1]}, R={self.multipath_status[2]}')
    
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
        
        # 基于用户意图、环境开放程度计算最优角度
        optimal_angle = self.calculate_optimal_direction(
            sector_angles, sector_openness
        )
        
        # 如果没有安全的移动方向，返回None表示应该停止
        if optimal_angle is None:
            self.get_logger().error('No safe direction found - emergency stop required')
            return None
        
        # 应用平滑滤波，避免突然的方向变化
        self.continuous_target_angle = (
            self.intent_smoothing_factor * optimal_angle + 
            (1 - self.intent_smoothing_factor) * self.continuous_target_angle
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
        consistency_score = max(0.0, 1.0 - distance_std / 1.0)

        # 综合开放度评分
        openness = (
            0.4 * distance_score +     # 40%权重给平均距离
            0.4 * safety_score +       # 40%权重给最小安全距离  
            0.2 * consistency_score    # 20%权重给一致性
        )
        
        return max(0.0, min(1.0, openness))

    def calculate_optimal_direction(self, sector_angles, sector_openness):
        """
        基于扇区开放度和用户意图计算最优移动方向
        """
        # 权重配置
        user_preference_weight = 0.6    # 用户意图权重
        environment_weight = 0.4        # 环境开放度权重

        # 将用户意图转换为偏好角度
        user_angle_map = {0: math.pi/4, 1: 0.0, 2: -math.pi/4}
        preferred_angle = user_angle_map.get(self.user_direction, 0.0)
        
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
            
            # 综合评分
            total_score = (
                user_preference_weight * user_score + 
                environment_weight * env_score
            )
            
            sector_scores.append(total_score)
        
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
        
        # 检测前方是否有近距离障碍物，如果有则增强侧向避障
        front_obstacle_detected = False
        min_front_distance = float('inf')

        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
            angle = angle_min + i * angle_increment
            
            # 检查前方±30度范围内的障碍物
            if abs(angle) < math.pi/6 and dist < 1.5:  # 前方1.5米内有障碍
                front_obstacle_detected = True
                min_front_distance = min(min_front_distance, dist)
            
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

        # 如果前方有障碍物，增强用户意图方向的侧向分量
        if front_obstacle_detected:
            avoidance_boost = 0.3 * (2.0 - min_front_distance)  # 距离越近，增强越多
            if self.user_direction == 0:  # 用户想要左转
                att_force_y += avoidance_boost  # 增强左转
                self.get_logger().debug(f'Front obstacle detected at {min_front_distance:.2f}m, boosting LEFT by {avoidance_boost:.2f}')
            elif self.user_direction == 2:  # 用户想要右转
                att_force_y -= avoidance_boost  # 增强右转
                self.get_logger().debug(f'Front obstacle detected at {min_front_distance:.2f}m, boosting RIGHT by {avoidance_boost:.2f}')
            # 如果用户想要前进但前方有障碍，选择开放度更好的一侧
            elif self.user_direction == 1:
                # 检查左右两侧的空间
                left_space = self.check_lateral_space(ranges, angle_min, angle_increment, math.pi/6, math.pi/3)
                right_space = self.check_lateral_space(ranges, angle_min, angle_increment, -math.pi/3, -math.pi/6)
                
                if left_space > right_space + 0.3:  # 左侧明显更开阔
                    att_force_y += avoidance_boost
                    self.get_logger().debug(f'Front obstacle, auto-choosing LEFT (space: L={left_space:.2f}, R={right_space:.2f})')
                elif right_space > left_space + 0.3:  # 右侧明显更开阔
                    att_force_y -= avoidance_boost
                    self.get_logger().debug(f'Front obstacle, auto-choosing RIGHT (space: L={left_space:.2f}, R={right_space:.2f})')

        # 合并吸引力和排斥力
        total_force_x = att_force_x + rep_force_x
        total_force_y = att_force_y + rep_force_y

        # 根据路径评估结果和势场力决定是否停止
        should_stop = False
        wheelchair_intent = ""
        
        # 计算势场强度来判断是否有足够的避障能力
        total_force_magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
        
        if emergency_stop_due_to_safety:
            should_stop = True
            wheelchair_intent = "EMERGENCY_STOP: No safe movement direction found"
        elif self.multipath_detected:
            # 多路径情况：停止等待用户选择
            should_stop = True
            wheelchair_intent = f"WAITING_FOR_USER: Multiple paths available - Left:{self.multipath_status[0]} Front:{self.multipath_status[1]} Right:{self.multipath_status[2]}"
        elif self.path_blocked and total_force_magnitude < 0.1:
            # 只有在路径被阻塞且势场力很小时才停止
            should_stop = True
            wheelchair_intent = "OBSTACLE_AVOIDANCE: Path blocked and insufficient force to navigate"
        else:
            # 正常移动状态 - 包含连续意图和避障信息
            direction_names = ["LEFT", "FORWARD", "RIGHT"]
            user_intent_name = direction_names[self.user_direction]
            
            # 将连续目标角度转换为度数
            target_angle_deg = math.degrees(continuous_goal_angle)
            
            # 检查是否正在进行避障
            if self.path_blocked and total_force_magnitude >= 0.1:
                # 路径被阻塞但势场力足够进行避障
                wheelchair_intent = (
                    f"FORCE_NAVIGATION: User intent={user_intent_name}, "
                    f"Force direction={math.degrees(math.atan2(total_force_y, total_force_x)):.1f}°, "
                    f"Force magnitude={total_force_magnitude:.2f}, "
                    f"Using potential field to navigate around obstacles"
                )
            elif abs(continuous_goal_angle) > math.pi/6:  # 偏离正前方超过30度
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
        intent_msg = String()
        intent_msg.data = wheelchair_intent
        self.wheelchair_intent_pub.publish(intent_msg)

        # 转换为线速度和角速度
        cmd = Twist()
        if should_stop:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            stop_reason = "Multiple paths" if self.multipath_detected else "Insufficient navigation force"
            self.get_logger().warn(f'Stopping: {stop_reason}')
        else:
            # 根据前方障碍情况调整速度
            if front_obstacle_detected and min_front_distance < 1.0:
                # 前方有近距离障碍，降低前进速度，增强转向能力
                max_linear_speed = 0.2 + 0.3 * (min_front_distance / 1.0)  # 距离越近速度越慢
                max_angular_speed = 0.7  # 增强转向能力
            else:
                # 正常情况
                max_linear_speed = 0.5
                max_angular_speed = 0.5
            
            cmd.linear.x = min(max_linear_speed, max(0.0, total_force_x))
            cmd.angular.z = min(max_angular_speed, max(-max_angular_speed, total_force_y))
        
        # 记录详细日志
        self.get_logger().debug(f'Path blocked: {self.path_blocked}, Multipath: {self.multipath_detected}')
        self.get_logger().debug(f'Emergency safety stop: {emergency_stop_due_to_safety}')
        self.get_logger().debug(f'Continuous target angle: {math.degrees(continuous_goal_angle):.1f}°')
        self.get_logger().debug(f'Should stop: {should_stop}')
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
