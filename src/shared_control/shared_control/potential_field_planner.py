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
        # ====== parameters =====
        self.goal_dist = 5.0  # 吸引点距离(m)
        self.obstacle_influence = 0.7  # 障碍物影响范围(m)
        self.repulsive_coef = 0.5  # 排斥力系数
        self.attractive_coef = 0.1  # 吸引力系数
        self.min_obstacle_dist = 0.7  # 最小安全距离(m)
        # 用户意图，默认前进
        self.user_direction = 1  # 0=左, 1=前, 2=右
        # 路径计算定时器
        self.timer = self.create_timer(0.1, self.calculate_path)
        # 激光数据存储
        self.laser_data = None
        self.get_logger().info('Potential Field Planner Node Initialized (Dynamic User Direction)')

    def user_dir_callback(self, msg):
        # 0=左, 1=前, 2=右
        if msg.data in [0, 1, 2]:
            self.user_direction = msg.data
            self.get_logger().info(f'Received user direction: {self.user_direction}')
        else:
            self.get_logger().warn(f'Invalid user direction: {msg.data}')
    
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

        # 记录最小距离
        min_dist = float('inf')
        front_blocked = False

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
            if abs(angle) < 0.3 and dist < self.min_obstacle_dist:
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
        if front_blocked:
            cmd.linear.x = 0.0
            self.get_logger().warn('Obstacle detected in front, stopping linear movement.')
        else:
            cmd.linear.x = min(0.5, total_force_x)
        cmd.angular.z = min(0.5, max(-0.5, total_force_y))
        self.get_logger().debug(f'Min distance: {min_dist:.2f}m, Total force: ({total_force_x:.2f}, {total_force_y:.2f})')
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
