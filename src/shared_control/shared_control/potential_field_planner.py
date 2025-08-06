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
        # 路径计算定时器
        self.timer = self.create_timer(0.1, self.calculate_path)
        # 激光数据存储
        self.laser_data = None
        self.get_logger().info('Potential Field Planner Node Initialized (Dynamic User Direction with Sliding Window)')

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

        # 动态设置吸引力方向
        # 0=左, 1=前, 2=右
        angle_map = {0: math.pi/4, 1: 0.0, 2: -math.pi/4}  # 左前、正前、右前
        goal_angle = angle_map.get(self.user_direction, 0.0)
        goal = Point()
        goal.x = self.goal_dist * math.cos(goal_angle)
        goal.y = self.goal_dist * math.sin(goal_angle)

        # 计算吸引力 (指向动态目标点)
        att_force_x = self.attractive_coef * goal.x
        att_force_y = self.attractive_coef * goal.y

        # 计算激光数据中的排斥力
        rep_force_x = 0.0
        rep_force_y = 0.0

        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

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
        
        if wall_collision_risk:
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
            # 正常移动状态
            direction_names = ["LEFT", "FORWARD", "RIGHT"]
            wheelchair_intent = f"MOVING: Following user direction {direction_names[self.user_direction]}"
        
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
        self.get_logger().debug(f'Should stop: {should_stop} ({stop_reason})')
        self.get_logger().debug(f'Wheelchair intent: {wheelchair_intent}')
        self.get_logger().debug(f'Total force: ({total_force_x:.2f}, {total_force_y:.2f})')
        self.get_logger().debug(f'Command: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}')
        self.path_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
