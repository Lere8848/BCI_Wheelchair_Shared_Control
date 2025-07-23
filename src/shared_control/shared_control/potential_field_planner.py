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
            '/scan',  # 根据实际情况修改话题名称
            self.laser_callback, 
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
        
        # ====== parameters =====
        self.goal_dist = 5.0  # 前方目标点距离(m)
        self.obstacle_influence = 0.7  # 障碍物影响范围(m)
        self.repulsive_coef = 0.5  # 排斥力系数
        self.attractive_coef = 0.1  # 吸引力系数
        self.min_obstacle_dist = 0.7  # 最小安全距离(m)
        
        # 路径计算定时器
        self.timer = self.create_timer(0.1, self.calculate_path)
        
        # 激光数据存储
        self.laser_data = None
        self.get_logger().info('Potential Field Planner Node Initialized')
    
    def laser_callback(self, msg):
        self.laser_data = msg
        # self.get_logger().debug('收到激光扫描数据')
    
    def calculate_path(self):
        if self.laser_data is None:
            self.get_logger().warn('Laser data not received yet')
            return

        # Create goal point in front of the wheelchair (relative to wheelchair's coordinate system)
        goal = Point()
        goal.x = self.goal_dist  # 前方x米处
        goal.y = 0.0
        
        # 计算吸引力 (指向目标点)
        att_force_x = self.attractive_coef * goal.x
        att_force_y = self.attractive_coef * goal.y
        
        # 计算激光数据中的排斥力
        rep_force_x = 0.0
        rep_force_y = 0.0
        
        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        # 记录最小距离
        min_dist = float('inf')
        front_blocked = False
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):  # 跳过无效测量值
                continue
                
            if dist < min_dist:
                min_dist = dist
            
            # 计算测量点的角度
            angle = angle_min + i * angle_increment
            
            # 如果距离小于影响范围，计算排斥力
            if dist < self.obstacle_influence:
                # 排斥力与距离成反比
                magnitude = self.repulsive_coef * (1.0 / dist - 1.0 / self.obstacle_influence)
                
                # 转换为笛卡尔坐标
                obstacle_x = dist * math.cos(angle)
                obstacle_y = dist * math.sin(angle)
                
                # 计算从障碍物指向机器人的方向
                if obstacle_x == 0 and obstacle_y == 0:
                    continue  # 避免除零错误
                
                # 障碍物坐标归一化
                norm = math.sqrt(obstacle_x**2 + obstacle_y**2)
                obstacle_x /= norm
                obstacle_y /= norm
                
                # 障碍物排斥力
                rep_force_x += magnitude * (-obstacle_x)  # 负号表示远离障碍物
                rep_force_y += magnitude * (-obstacle_y)
            
            # 检查前方是否被阻挡
            if abs(angle) < 0.3 and dist < self.min_obstacle_dist:  # 约±17度范围内
                front_blocked = True
        
        # 发布前方是否被阻挡的信息
        blocked_msg = Bool()
        blocked_msg.data = front_blocked
        self.obstacle_pub.publish(blocked_msg)
        
        # 合并吸引力和排斥力
        total_force_x = att_force_x + rep_force_x
        total_force_y = att_force_y + rep_force_y
        
        # 转换为线速度和角速度
        cmd = Twist()
        
        # 如果前方被阻挡，减小线速度
        if front_blocked:
            cmd.linear.x = 0.0
            self.get_logger().warn('Obstacle detected in front, stopping linear movement.')
        else:
            cmd.linear.x = min(0.5, total_force_x)  # 限制最大速度
            
        # 角速度正比于力的y分量 (负值表示向右转)
        cmd.angular.z = min(0.5, max(-0.5, total_force_y))  # 限制最大角速度
        
        # 记录日志
        self.get_logger().debug(f'Min distance: {min_dist:.2f}m, Total force: ({total_force_x:.2f}, {total_force_y:.2f})')
        self.get_logger().debug(f'Command: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}')

        # 发布命令
        self.path_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
