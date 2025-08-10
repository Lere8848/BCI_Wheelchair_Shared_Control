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

        # ====== 势场参数配置 =====
        self.obstacle_influence = 0.5  # 障碍物影响范围(m) - 超过此距离的障碍物不产生排斥力
        self.repulsive_coef = 0.4  # 排斥力系数 - 控制障碍物排斥力的强度
        self.attractive_coef = 0.5  # 吸引力系数 - 控制用户意图吸引力的强度
        
        # 用户意图相关参数
        self.user_direction = 1  # 当前用户方向意图: 0=左, 1=前, 2=右
        self.user_input_history = []  # 用户输入历史记录 - 用于计算输入频率
        self.max_history_time = 2.0  # 最大历史记录时间(秒) - 超过此时间的输入将被清除
        
        # 三大扇区配置 (左60°, 前60°, 右60°) - 与path_eval_node保持一致
        self.major_sectors = {
            0: (-math.pi/2, -math.pi/6),    # 左扇区：-90°到-30°
            1: (-math.pi/6, math.pi/6),     # 前扇区：-30°到30°
            2: (math.pi/6, math.pi/2)       # 右扇区：30°到90°
        }
        
        # 每个大扇区内的小扇区数量 - 用于精细化扇区分析
        self.sub_sectors_per_major = 6  # 每个60°大扇区分为6个10°小扇区
        
        # 动态吸引子参数
        self.min_attractor_distance = 0.5  # 最小吸引子距离(m) - 高频输入时的吸引子位置
        self.max_attractor_distance = 1.0  # 最大吸引子距离(m) - 低频输入时的吸引子位置
        self.current_attractor_pos = None   # 当前吸引子位置 - 存储最新计算的吸引子坐标
        
        # 来自路径评估节点的状态
        self.path_blocked = False
        self.multipath_detected = False
        self.multipath_status = [False, False, False]  # [左, 前, 右] 路径状态
        
        # 路径计算定时器
        self.timer = self.create_timer(0.1, self.calculate_path)
        # 激光数据存储
        self.laser_data = None
        self.get_logger().info('Potential Field Planner Node Initialized')

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
    


    def find_optimal_attractor_position(self, ranges, angle_min, angle_increment):
        """
        基于用户意图和path_eval结果，找到最优吸引子位置
        使用简化逻辑，依赖path_eval提供的路径可行性分析
        """
        # 获取用户偏好方向的角度范围
        user_sector_start, user_sector_end = self.major_sectors[self.user_direction]
        
        # 在用户偏好扇区内寻找开放区域
        best_angle = (user_sector_start + user_sector_end) / 2  # 默认选择扇区中心
        best_distance = float('inf')
        
        # 扫描用户偏好扇区，寻找最开放的方向
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
                
            angle = angle_min + i * angle_increment
            
            # 检查是否在用户偏好扇区内
            if user_sector_start <= angle <= user_sector_end:
                if dist > best_distance:
                    best_distance = dist
                    best_angle = angle
        
        # 如果用户偏好方向被path_eval判定为不可行，寻找备选方案
        if not self.multipath_status[self.user_direction]:
            self.get_logger().warn(f'User preferred direction not available according to path_eval, searching alternatives')
            
            # 按优先级寻找可行方向：前 > 左/右
            alternative_directions = [1, 0, 2] if self.user_direction != 1 else [0, 2]
            
            for alt_dir in alternative_directions:
                if self.multipath_status[alt_dir]:
                    # 使用备选方向
                    alt_start, alt_end = self.major_sectors[alt_dir]
                    best_angle = (alt_start + alt_end) / 2
                    
                    # 在备选扇区内寻找最开放的点
                    for i, dist in enumerate(ranges):
                        if not math.isfinite(dist):
                            continue
                        angle = angle_min + i * angle_increment
                        if alt_start <= angle <= alt_end and dist > best_distance:
                            best_distance = dist
                            best_angle = angle
                    
                    self.get_logger().info(f'Using alternative direction: {["left", "forward", "right"][alt_dir]}')
                    break
        
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
        
        # 确保吸引子距离不超过该方向的最大可见距离
        if math.isfinite(best_distance):
            attractor_distance = min(attractor_distance, best_distance * 0.8)
        
        # 计算吸引子位置
        attractor_x = attractor_distance * math.cos(best_angle)
        attractor_y = attractor_distance * math.sin(best_angle)

        self.get_logger().debug(f'Attractor position: Angle={math.degrees(best_angle):.1f}°, Distance={attractor_distance:.2f}m')

        return {
            'x': attractor_x,
            'y': attractor_y,
            'angle': best_angle,
            'distance': attractor_distance,
            'openness': min(1.0, best_distance / 3.0) if math.isfinite(best_distance) else 0.0,
            'major_direction': self.user_direction
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
            if safe_distance > 0.5:  # 至少保持0.5米距离
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
    
    def calculate_path(self):
        """
        势场路径计算方法：动态吸引子 + 排斥力
        依赖path_eval_node提供的路径可行性分析
        """
        if self.laser_data is None:
            self.get_logger().warn('Laser data not received')
            return

        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        # 1. 基于用户意图和path_eval结果找到最优吸引子位置
        attractor_pos = self.find_optimal_attractor_position(ranges, angle_min, angle_increment)
        
        # 2. 检查吸引子位置的可见性和安全性
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
        
        # 3. 更新当前吸引子位置
        self.current_attractor_pos = attractor_pos
        
        # 4. 计算吸引力（指向动态吸引子）
        att_force_x = self.attractive_coef * attractor_pos['x']
        att_force_y = self.attractive_coef * attractor_pos['y']
        
        # 5. 计算排斥力（来自所有障碍物）
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
        
        # 6. 合并吸引力和排斥力
        total_force_x = att_force_x + rep_force_x
        total_force_y = att_force_y + rep_force_y
        
        # 7. 检查是否需要停止（基于势场强度，多路径由融合节点处理）
        total_force_magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
        
        should_stop = False
        stop_reason = ""
        
        if total_force_magnitude < 0.05:
            should_stop = True
            stop_reason = "Attractive force too weak"

        # 8. 生成运动命令
        cmd = Twist()
        
        if should_stop:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            wheelchair_intent = f"STOP: {stop_reason}"
        else:
            # 将势场力转换为运动命令
            # X方向力控制前进速度，Y方向力控制转向
            
            # 前进速度：基于X方向力，限制在合理范围内
            cmd.linear.x = max(0.0, min(0.5, total_force_x))
            
            # 转向速度：基于Y方向力
            cmd.angular.z = max(-0.4, min(0.4, total_force_y))
            
            # 根据距离障碍物的远近调整速度
            min_obstacle_dist = float('inf')
            for dist in ranges:
                if math.isfinite(dist):
                    min_obstacle_dist = min(min_obstacle_dist, dist)
            
            if min_obstacle_dist < 1.0:
                # 接近障碍物时降低速度
                speed_factor = min_obstacle_dist / 1.0
                cmd.linear.x *= speed_factor
                cmd.angular.z *= 1.1  # 增强转向能力
            
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
        
        # 9. 发布命令和意图
        self.path_pub.publish(cmd)
        
        intent_msg = String()
        intent_msg.data = wheelchair_intent
        self.wheelchair_intent_pub.publish(intent_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
