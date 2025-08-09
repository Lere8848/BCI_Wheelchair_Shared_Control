# potential_field_planner.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Int8, Int8MultiArray, String
import numpy as np
import math
import rclpy.duration

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        # 订阅激光雷达数据
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        # 订阅用户意图（0=左, 1=前, 2=右）
        self.user_dir_sub = self.create_subscription(Int8, '/user_cmd', self.user_dir_callback, 10)
        # 订阅路径评估结果
        self.path_blocked_sub = self.create_subscription(Bool, '/path_blocked', self.path_blocked_callback, 10)
        self.multipath_sub = self.create_subscription(Int8MultiArray, '/multipath_detected', self.multipath_callback, 10)
        # 发布计算出的路径指令
        self.path_pub = self.create_publisher(Twist, '/auto_cmd_vel', 10)
        # 发布轮椅意图给Unity端
        self.wheelchair_intent_pub = self.create_publisher(String, '/wheelchair_intent', 10)

        # ====== 新的动态吸引子势场参数 =====
        self.obstacle_influence = 0.6  # 障碍物影响范围(m)
        self.repulsive_coef = 0.5  # 排斥力系数
        self.attractive_coef = 0.6  # 吸引力系数
        
        # 用户意图相关参数
        self.user_direction = 1  # 0=左, 1=前, 2=右
        self.user_input_history = []  # 用户输入历史记录
        self.max_history_time = 2.0  # 最大历史记录时间(秒)
        
        # 三大扇区配置 (左60°, 前60°, 右60°)
        self.major_sectors = {
            0: (math.pi/6, math.pi/2),      # 左扇区：30°到90°
            1: (-math.pi/6, math.pi/6),     # 前扇区：-30°到30°
            2: (-math.pi/2, -math.pi/6)     # 右扇区：-90°到-30°
        }
        
        # 每个大扇区内的小扇区数量
        self.sub_sectors_per_major = 6  # 每个60°大扇区分为6个10°小扇区
        
        # 动态吸引子参数
        self.min_attractor_distance = 0.5  # 最小吸引子距离(m)
        self.max_attractor_distance = 1.0  # 最大吸引子距离(m)
        self.current_attractor_pos = None   # 当前吸引子位置
        
        # 来自路径评估节点的状态（保留用于安全检查）
        self.path_blocked = False
        self.multipath_detected = False
        self.multipath_status = [False, False, False]
        
        # 路径计算定时器
        self.timer = self.create_timer(0.1, self.calculate_path)
        # 激光数据存储
        self.laser_data = None
        self.get_logger().info('Potential Field Planner Node Initialized (Focuses on Force Calculation)')

    def user_dir_callback(self, msg):
        """接收用户意图并记录历史"""
        if msg.data in [0, 1, 2]:
            # 记录用户输入历史（时间戳 + 方向）
            current_time = self.get_clock().now()
            self.user_input_history.append((current_time, msg.data))
            
            # 清除超过2秒的历史记录
            cutoff_time = current_time - rclpy.duration.Duration(seconds=self.max_history_time)
            self.user_input_history = [
                (t, d) for t, d in self.user_input_history 
                if t >= cutoff_time
            ]
            
            self.user_direction = msg.data
            self.get_logger().info(f'User intent: {["left", "forward", "right"][msg.data]}, Input count within 2s: {len(self.user_input_history)}')
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
    
    def analyze_sectors_openness(self, ranges, angle_min, angle_increment):
        """
        分析三大扇区的开放程度
        返回: dict {方向: {小扇区开放度列表}}
        """
        sector_analysis = {}
        
        for direction, (start_angle, end_angle) in self.major_sectors.items():
            sector_width = (end_angle - start_angle) / self.sub_sectors_per_major
            sub_sector_openness = []
            
            # 分析每个小扇区
            for i in range(self.sub_sectors_per_major):
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
        计算小扇区的开放程度
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
        if min_distance < 0.8:
            safety_penalty = 0.8  # 大幅降低
        elif min_distance < 1.5:
            safety_penalty = 0.5  # 中等降低
        else:
            safety_penalty = 0.0  # 无惩罚
        
        final_openness = base_openness * (1.0 - safety_penalty)
        return max(0.0, min(1.0, final_openness))

    def find_optimal_attractor_position(self, sector_analysis):
        """
        基于用户意图和扇区分析，找到最优吸引子位置
        """
        # 获取用户偏好的大扇区
        user_preferred_sector = sector_analysis[self.user_direction]
        
        # 在偏好扇区中找到开放度最高的小扇区
        best_sub_sector = max(user_preferred_sector, key=lambda x: x['openness'])
        
        # 如果用户偏好扇区的最佳开放度太低，考虑其他扇区
        if best_sub_sector['openness'] < 0.3:
            self.get_logger().warn(f'User preferred sector openness is low ({best_sub_sector["openness"]:.2f}), searching for alternatives.')

            # 寻找所有扇区中开放度最高的
            all_sub_sectors = []
            for direction, sub_sectors in sector_analysis.items():
                for sub_sector in sub_sectors:
                    sub_sector['major_direction'] = direction
                    all_sub_sectors.append(sub_sector)
            
            # 按开放度排序
            all_sub_sectors.sort(key=lambda x: x['openness'], reverse=True)
            
            # 优先考虑与用户意图相近的扇区
            for sub_sector in all_sub_sectors:
                if sub_sector['openness'] > 0.4:  # 足够的开放度
                    best_sub_sector = sub_sector
                    best_sub_sector['major_direction'] = sub_sector['major_direction']
                    self.get_logger().info(f'Selected alternative direction: {["left", "forward", "right"][best_sub_sector["major_direction"]]} sector')
                    break
        else:
            best_sub_sector['major_direction'] = self.user_direction
        
        # 计算吸引子距离（基于用户输入频率）
        input_frequency = len(self.user_input_history)
        if input_frequency <= 2:
            attractor_distance = self.max_attractor_distance  # 低频输入，远距离
        elif input_frequency >= 8:
            attractor_distance = self.min_attractor_distance  # 高频输入，近距离
        else:
            # 线性插值
            ratio = (input_frequency - 2) / (8 - 2)
            attractor_distance = self.max_attractor_distance - ratio * (self.max_attractor_distance - self.min_attractor_distance)
        
        # 计算吸引子位置
        attractor_angle = best_sub_sector['center_angle']
        attractor_x = attractor_distance * math.cos(attractor_angle)
        attractor_y = attractor_distance * math.sin(attractor_angle)

        self.get_logger().debug(f'Attractor position: Angle={math.degrees(attractor_angle):.1f}°, Distance={attractor_distance:.2f}m, Openness={best_sub_sector["openness"]:.2f}')

        return {
            'x': attractor_x,
            'y': attractor_y,
            'angle': attractor_angle,
            'distance': attractor_distance,
            'openness': best_sub_sector['openness'],
            'major_direction': best_sub_sector['major_direction']
        }

    def check_attractor_visibility(self, attractor_pos, ranges, angle_min, angle_increment):
        """
        检查吸引子位置是否在激光探测范围内且无障碍物阻挡
        """
        attractor_angle = attractor_pos['angle']
        attractor_distance = attractor_pos['distance']
        
        # 找到最接近吸引子角度的激光射线
        target_ray_index = int((attractor_angle - angle_min) / angle_increment)
        target_ray_index = max(0, min(target_ray_index, len(ranges) - 1))
        
        measured_distance = ranges[target_ray_index]
        
        # 检查该方向是否有足够的可见距离
        if math.isfinite(measured_distance) and measured_distance > attractor_distance:
            return True  # 吸引子位置可见且无阻挡
        else:
            # 吸引子位置被障碍物阻挡，需要调整
            # 将吸引子拉近到安全距离
            safe_distance = min(measured_distance * 0.8, attractor_distance)
            if safe_distance > 1.0:  # 至少保持1米距离
                attractor_pos['distance'] = safe_distance
                attractor_pos['x'] = safe_distance * math.cos(attractor_angle)
                attractor_pos['y'] = safe_distance * math.sin(attractor_angle)
                self.get_logger().debug(f'Attractor position blocked by obstacle, adjusted distance to {safe_distance:.2f}m')
                return True
            else:
                self.get_logger().warn('Attractor position severely blocked, unable to place safely')
                return False

    def laser_callback(self, msg):
        self.laser_data = msg
        # self.get_logger().debug('收到激光扫描数据')
    
    def calculate_path(self):
        """
        新的势场路径计算方法：动态吸引子 + 排斥力
        """
        if self.laser_data is None:
            self.get_logger().warn('Laser data not received')
            return

        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        # 1. 分析所有扇区的开放程度
        sector_analysis = self.analyze_sectors_openness(ranges, angle_min, angle_increment)
        
        # 2. 根据用户意图和扇区分析找到最优吸引子位置
        attractor_pos = self.find_optimal_attractor_position(sector_analysis)
        
        # 3. 检查吸引子位置的可见性和安全性
        attractor_valid = self.check_attractor_visibility(attractor_pos, ranges, angle_min, angle_increment)
        
        if not attractor_valid:
            # 如果无法安全放置吸引子，停止运动
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.path_pub.publish(cmd)
            
            intent_msg = String()
            intent_msg.data = "STOP: Unable to place attractor safely"
            self.wheelchair_intent_pub.publish(intent_msg)
            return
        
        # 4. 更新当前吸引子位置
        self.current_attractor_pos = attractor_pos
        
        # 5. 计算吸引力（指向动态吸引子）
        att_force_x = self.attractive_coef * attractor_pos['x']
        att_force_y = self.attractive_coef * attractor_pos['y']
        
        # 6. 计算排斥力（来自所有障碍物）
        rep_force_x = 0.0
        rep_force_y = 0.0
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist) or dist > self.obstacle_influence:
                continue
                
            angle = angle_min + i * angle_increment
            
            # 计算排斥力
            repulsive_magnitude = self.repulsive_coef * (1.0 / dist - 1.0 / self.obstacle_influence)
            
            # 障碍物在激光坐标系中的位置
            obstacle_x = dist * math.cos(angle)
            obstacle_y = dist * math.sin(angle)
            
            # 从障碍物指向轮椅的单位向量（排斥力方向）
            if obstacle_x == 0 and obstacle_y == 0:
                continue
                
            norm = math.sqrt(obstacle_x**2 + obstacle_y**2)
            rep_force_x += repulsive_magnitude * (-obstacle_x / norm)
            rep_force_y += repulsive_magnitude * (-obstacle_y / norm)
        
        # 7. 合并吸引力和排斥力
        total_force_x = att_force_x + rep_force_x
        total_force_y = att_force_y + rep_force_y
        
        # 8. 检查是否需要停止（基于势场强度，多路径由融合节点处理）
        total_force_magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
        
        should_stop = False
        stop_reason = ""
        
        if total_force_magnitude < 0.05:
            should_stop = True
            stop_reason = "Attractive force too weak"
        # 移除多路径自动停止逻辑，让融合节点处理
        # elif self.multipath_detected and not self.user_input_history:
        #     should_stop = True
        #     stop_reason = "Multiple paths detected, waiting for user input"

        # 9. 生成运动命令
        cmd = Twist()
        
        if should_stop:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            wheelchair_intent = f"STOP: {stop_reason}"
        else:
            # 将势场力转换为运动命令
            # X方向力控制前进速度，Y方向力控制转向
            
            # 前进速度：基于X方向力，限制在合理范围内
            cmd.linear.x = max(0.0, min(0.6, total_force_x))
            
            # 转向速度：基于Y方向力
            cmd.angular.z = max(-0.8, min(0.8, total_force_y))
            
            # 根据距离障碍物的远近调整速度
            min_obstacle_dist = float('inf')
            for dist in ranges:
                if math.isfinite(dist):
                    min_obstacle_dist = min(min_obstacle_dist, dist)
            
            if min_obstacle_dist < 1.0:
                # 接近障碍物时降低速度
                speed_factor = min_obstacle_dist / 1.0
                cmd.linear.x *= speed_factor
                cmd.angular.z *= 1.2  # 增强转向能力
            
            # 生成意图描述
            direction_names = ["Left", "Forward", "Right"]
            user_intent_name = direction_names[self.user_direction]
            attractor_direction_name = direction_names[attractor_pos['major_direction']]
            
            wheelchair_intent = (
                f"DYNAMIC_NAVIGATION: User intent={user_intent_name}, "
                f"Attractor direction={attractor_direction_name}, "
                f"Attractor distance={attractor_pos['distance']:.1f}m, "
                f"Openness={attractor_pos['openness']:.2f}, "
                f"Input frequency={len(self.user_input_history)}"
            )
        
        # 10. 发布命令和意图
        self.path_pub.publish(cmd)
        
        intent_msg = String()
        intent_msg.data = wheelchair_intent
        self.wheelchair_intent_pub.publish(intent_msg)
        
        # 11. 详细日志
        self.get_logger().debug(f'Attractor position: ({attractor_pos["x"]:.2f}, {attractor_pos["y"]:.2f})')
        self.get_logger().debug(f'Attractive force: ({att_force_x:.2f}, {att_force_y:.2f})')
        self.get_logger().debug(f'Repulsive force: ({rep_force_x:.2f}, {rep_force_y:.2f})')
        self.get_logger().debug(f'Total force: ({total_force_x:.2f}, {total_force_y:.2f})')
        self.get_logger().debug(f'Command: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}')
        self.get_logger().info(wheelchair_intent)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
