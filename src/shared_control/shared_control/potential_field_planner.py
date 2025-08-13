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
        # Subscribe to Unity data
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
 
        self.user_dir_sub = self.create_subscription(Int8, '/user_cmd', self.user_dir_callback, 10) # user intent (0=left, 1=right, 2=forward)

        self.path_blocked_sub = self.create_subscription(Bool, '/path_blocked', self.path_blocked_callback, 10)
        self.multipath_sub = self.create_subscription(Int8MultiArray, '/multipath_detected', self.multipath_callback, 10)

        self.path_pub = self.create_publisher(Twist, '/auto_cmd_vel', 10) # Publish calculated path commands
        self.wheelchair_intent_pub = self.create_publisher(String, '/wheelchair_intent', 10) # to unity, Publish wheelchair intent

        # to Unity for visualization - three direction attractors
        self.attractor_pos_pub = self.create_publisher(Point, '/user_attractor_pos', 10) # User selected attractor position
        self.left_attractor_pub = self.create_publisher(Point, '/left_attractor_pos', 10)    # Left direction attractor position
        self.forward_attractor_pub = self.create_publisher(Point, '/forward_attractor_pos', 10) # Forward direction attractor position
        self.right_attractor_pub = self.create_publisher(Point, '/right_attractor_pos', 10)   # Right direction attractor position

        # ====== Potential field parameter configuration =====
        self.obstacle_influence = 0.4  # Obstacle influence range (m) - obstacles beyond this distance don't generate repulsive force
        self.repulsive_coef = 0.3  # Repulsive force coefficient - controls strength of obstacle repulsive force
        self.attractive_coef = 0.5  # Attractive force coefficient - controls strength of user intent attractive force
        
        # User intent related parameters
        self.user_direction = 1  # Current user direction intent: 0=left, 1=forward, 2=right (internal mapping), default=forward
        self.user_input_history = []  # User input history - used to calculate input frequency
        self.max_history_time = 2.0  # Maximum history record time (seconds) - inputs older than this will be cleared
        
        # Three major sector configuration (left 60°, forward 60°, right 60°) - Unity coordinate system: left-right flipped
        self.major_sectors = {
            0: (-math.pi/2, -math.pi/6),        # Left sector: -90° to -30° (Unity negative angle = left side)
            1: (-math.pi/6, math.pi/6),         # Forward sector: -30° to 30° (Unity 0° = forward)
            2: (math.pi/6, math.pi/2)           # Right sector: 30° to 90° (Unity positive angle = right side)
        }
        
        # Number of sub-sectors within each major sector - used for refined sector analysis
        self.sub_sectors_per_major = 6  # Each 60° major sector is divided into 6 sub-sectors of 10° each
        
        # Dynamic attractor parameters
        self.min_attractor_distance = 0.9  # Minimum attractor distance (m) - attractor position for high frequency input
        self.max_attractor_distance = 1.8  # Maximum attractor distance (m) - attractor position for low frequency input
        self.current_attractor_pos = None   # Current attractor position - stores latest calculated attractor coordinates
        
        # Lidar offset parameters relative to wheelchair center (Unity coordinate system and ROS2 coordinate system conversion)
        self.lidar_offset_x = 0.464   # Unity Z-axis -> ROS X-axis (forward offset)
        self.lidar_offset_y = 0       # Unity X-axis -> ROS Y-axis (left offset)  
        self.lidar_rotation_offset = math.radians(-25)  # Y-axis rotation offset (radians) - Unity's -25 degrees converted to radians
        
        # Status from path evaluation node
        self.path_blocked = False
        self.multipath_detected = False
        self.multipath_status = [False, False, False]  # [left, forward, right] path status
        
        # Path calculation timer
        self.timer = self.create_timer(0.1, self.calculate_path)
        # Laser data storage
        self.laser_data = None
        self.get_logger().info('Potential Field Planner Node Initialized')

    def map_BCI_intent_to_internal(self, user_cmd):
        """
        Map BCI command format (LRF) to internal processing format (LFR)
        """
        if user_cmd == 0:  # left -> left
            return 0
        elif user_cmd == 1:  # right -> right
            return 2
        elif user_cmd == 2:  # forward -> forward
            return 1
        else:
            return None  # invalid command

    def user_dir_callback(self, msg):
        """Receive user intent and record history"""
        if msg.data in [0, 1, 2]:
            # Map user command to internal direction format
            internal_direction = self.map_BCI_intent_to_internal(msg.data)
            if internal_direction is None:
                self.get_logger().warn(f'Invalid user direction mapping: {msg.data}')
                return
            
            # Record user input history (timestamp + internal direction)
            current_time = self.get_clock().now()
            self.user_input_history.append((current_time, internal_direction))
            
            # Clear history records older than 2 seconds
            cutoff_time = current_time - rclpy.duration.Duration(seconds=self.max_history_time)
            self.user_input_history = [
                (t, d) for t, d in self.user_input_history 
                if t >= cutoff_time
            ]
            
            self.user_direction = internal_direction
            # Log with both user command and internal mapping
            user_cmd_names = ["left", "right", "forward"]  # 0=left, 1=right, 2=forward
            internal_names = ["left", "forward", "right"]  # 0=left, 1=forward, 2=right
            self.get_logger().info(f'User intent: {user_cmd_names[msg.data]}, Input count within 2s: {len(self.user_input_history)}')
        else:
            self.get_logger().warn(f'Invalid user direction: {msg.data}')
    
    def path_blocked_callback(self, msg):
        """receive blocked status from path evaluation node"""
        self.path_blocked = msg.data
        if self.path_blocked:
            self.get_logger().debug('Path blocked signal received from PathEvalNode')
    
    def multipath_callback(self, msg):
        """receive multipath detection results from path evaluation node"""
        if len(msg.data) == 3:
            self.multipath_status = [bool(x) for x in msg.data]
            self.multipath_detected = sum(self.multipath_status) >= 2
            if self.multipath_detected:
                self.get_logger().debug(f'Multipath detected: L={self.multipath_status[0]}, F={self.multipath_status[1]}, R={self.multipath_status[2]}')
    
    def find_optimal_attractor_positions_all_directions(self, ranges, angle_min, angle_increment):
        """
        calculate optimal attractor positions for three directions
        return dictionary containing attractor information for three directions
        """
        all_attractors = {}
        
        # calculate user input frequency (for distance calculation in all directions)
        input_frequency = len(self.user_input_history)
        if input_frequency <= 2:
            attractor_distance = self.min_attractor_distance  # low frequency input, close distance
        elif input_frequency >= 8:
            attractor_distance = self.max_attractor_distance  # high frequency input, far distance
        else:
            # linear interpolation: higher frequency, farther distance
            ratio = (input_frequency - 2) / (8 - 2)
            attractor_distance = self.min_attractor_distance + ratio * (self.max_attractor_distance - self.min_attractor_distance)
        
        # calculate optimal attractor position for each of the three major directions
        for direction in [0, 1, 2]:  # 0=left, 1=forward, 2=right
            # get angle range for this direction
            sector_start, sector_end = self.major_sectors[direction]
            
            # check if this direction is feasible
            if not self.multipath_status[direction]:
                all_attractors[direction] = None
                continue
            
            # decompose into sub-sectors within this major sector, find sub-sector with highest openness
            sector_width = (sector_end - sector_start) / self.sub_sectors_per_major
            best_openness = 0.0
            best_angle = (sector_start + sector_end) / 2  # default select sector center
            best_distance = 0.0
            
            # analyze each sub-sector openness
            for i in range(self.sub_sectors_per_major):
                sub_start = sector_start + i * sector_width
                sub_end = sector_start + (i + 1) * sector_width
                sub_center = (sub_start + sub_end) / 2
                
                # calculate openness of this sub-sector
                distances_in_subsector = []
                for j, dist in enumerate(ranges):
                    if not math.isfinite(dist):
                        continue
                    angle = angle_min + j * angle_increment
                    if sub_start <= angle <= sub_end:
                        distances_in_subsector.append(dist)
                
                if len(distances_in_subsector) == 0:
                    continue
                    
                # calculate sub-sector openness
                distances = np.array(distances_in_subsector)
                avg_distance = np.mean(distances)
                min_distance = np.min(distances)
                
                # basic openness: based on average distance
                base_openness = min(1.0, avg_distance / 3.0)
                
                # safety penalty: if minimum distance too small, greatly reduce openness
                if min_distance < 0.8:
                    safety_penalty = 0.8
                elif min_distance < 1.5:
                    safety_penalty = 0.5
                else:
                    safety_penalty = 0.0
                
                final_openness = base_openness * (1.0 - safety_penalty)
                final_openness = max(0.0, min(1.0, final_openness))
                
                # select sub-sector with highest openness
                if final_openness > best_openness:
                    best_openness = final_openness
                    best_angle = sub_center
                    best_distance = avg_distance
            
            # ensure attractor distance does not exceed maximum visible distance for this direction
            final_attractor_distance = attractor_distance
            if best_distance > 0:
                final_attractor_distance = min(attractor_distance, best_distance * 0.8)
            
            # calculate attractor position in laser scanner coordinate system center
            lidar_x = final_attractor_distance * math.cos(best_angle)
            lidar_y = final_attractor_distance * math.sin(best_angle)
            
            # convert to wheelchair center coordinate system
            wheelchair_x, wheelchair_y = self.transform_lidar_to_wheelchair_center(lidar_x, lidar_y)

            all_attractors[direction] = {
                'x': wheelchair_x,  # X coordinate in wheelchair center coordinate system
                'y': wheelchair_y,  # Y coordinate in wheelchair center coordinate system
                'angle': best_angle,
                'distance': final_attractor_distance,
                'openness': best_openness,
                'major_direction': direction
            }
        
        return all_attractors

    def transform_lidar_to_wheelchair_center(self, lidar_x, lidar_y):
        """
        convert position from lidar coordinate system to wheelchair center coordinate system
        consider lidar position offset and rotation offset
        
        Args:
            lidar_x, lidar_y: position in lidar coordinate system
        Returns:
            wheelchair_x, wheelchair_y: position in wheelchair center coordinate system
        """
        # 1. apply rotation transformation (compensate for laser scanner rotation offset)
        cos_rot = math.cos(-self.lidar_rotation_offset)  # use negative angle for reverse rotation
        sin_rot = math.sin(-self.lidar_rotation_offset)
        
        # rotation matrix transformation
        rotated_x = lidar_x * cos_rot - lidar_y * sin_rot
        rotated_y = lidar_x * sin_rot + lidar_y * cos_rot
        
        # 2. apply position offset (move laser scanner coordinate origin to wheelchair center)
        wheelchair_x = rotated_x + self.lidar_offset_x
        wheelchair_y = rotated_y + self.lidar_offset_y
        
        return wheelchair_x, wheelchair_y

    def publish_all_attractor_positions(self, all_attractors):
        """
        publish attractor positions for three directions to Unity
        """
        unity_scale_factor = 5  # Unity will have a 0.2 scale factor so multiply by 5 here to ensure Attractor position is correct

        # publish left direction attractor
        left_msg = Point()
        if all_attractors[0] is not None:
            left_msg.x = float((all_attractors[0]['y'] )* unity_scale_factor)  # ROS_Y -> Unity_X
            left_msg.y = 0.1 * unity_scale_factor  # Unity height fixed
            left_msg.z = float((all_attractors[0]['x'] ) * unity_scale_factor)  # ROS_X -> Unity_Z
        else:
            # invalid position
            left_msg.x = float('inf')
            left_msg.y = float('inf')
            left_msg.z = float('inf')
        self.left_attractor_pub.publish(left_msg)

        # publish forward direction attractor
        forward_msg = Point()
        if all_attractors[1] is not None:
            forward_msg.x = float((all_attractors[1]['y'] ) * unity_scale_factor)  # ROS_Y -> Unity_X
            forward_msg.y = 0.1 * unity_scale_factor  # Unity height fixed
            forward_msg.z = float((all_attractors[1]['x'] ) * unity_scale_factor)  # ROS_X -> Unity_Z
        else:
            # invalid position
            forward_msg.x = float('inf')
            forward_msg.y = float('inf')
            forward_msg.z = float('inf')
        self.forward_attractor_pub.publish(forward_msg)

        # publish right direction attractor
        right_msg = Point()
        if all_attractors[2] is not None:
            right_msg.x = float((all_attractors[2]['y'] ) * unity_scale_factor)  # ROS_Y -> Unity_X
            right_msg.y = 0.1 * unity_scale_factor  # Unity height fixed
            right_msg.z = float((all_attractors[2]['x'] ) * unity_scale_factor)  # ROS_X -> Unity_Z
        else:
            # invalid position
            right_msg.x = float('inf')
            right_msg.y = float('inf')
            right_msg.z = float('inf')
        self.right_attractor_pub.publish(right_msg)

    def check_attractor_visibility(self, attractor_pos, ranges, angle_min, angle_increment):
        """
        check if attractor position is within laser detection range and unobstructed by obstacles
        """
        attractor_angle = attractor_pos['angle']
        attractor_distance = attractor_pos['distance']
        
        # find laser ray closest to attractor angle
        target_ray_index = int((attractor_angle - angle_min) / angle_increment)
        target_ray_index = max(0, min(target_ray_index, len(ranges) - 1))
        
        measured_distance = ranges[target_ray_index]
        
        # check if this direction has sufficient visible distance
        if math.isfinite(measured_distance) and measured_distance > attractor_distance:
            return True  # attractor position is visible and unobstructed
        else:
            # attractor position is blocked by obstacle, needs adjustment
            # pull attractor closer to safe distance
            safe_distance = min(measured_distance * 0.8, attractor_distance)
            if safe_distance > 0.6:  # maintain at least 0.6 meter distance
                attractor_pos['distance'] = safe_distance
                # recalculate adjusted position (considering coordinate system conversion)
                lidar_x = safe_distance * math.cos(attractor_angle)
                lidar_y = safe_distance * math.sin(attractor_angle)
                wheelchair_x, wheelchair_y = self.transform_lidar_to_wheelchair_center(lidar_x, lidar_y)
                attractor_pos['x'] = wheelchair_x
                attractor_pos['y'] = wheelchair_y
                self.get_logger().debug(f'Attractor position blocked by obstacle, adjusted distance to {safe_distance:.2f}m')
                return True
            else:
                self.get_logger().warn('Attractor position severely blocked, unable to place safely')
                return False

    def laser_callback(self, msg):
        self.laser_data = msg
    
    def calculate_path(self):
        """
        potential field path calculation method: dynamic attractor + repulsive force
        depends on path feasibility analysis provided by path_eval_node
        """
        if self.laser_data is None:
            self.get_logger().warn('Laser data not received')
            return

        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        # 1. calculate optimal attractor position for three directions
        all_attractors = self.find_optimal_attractor_positions_all_directions(ranges, angle_min, angle_increment)
        
        # 2. publish all three direction attractor positions to Unity
        self.publish_all_attractor_positions(all_attractors)
        
        # 3. get attractor position for user selected direction for execution
        attractor_pos = all_attractors[self.user_direction]
        
        # if user preferred direction is not feasible, keep idle and wait for new command
        if attractor_pos is None:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.path_pub.publish(cmd)
            
            # publish invalid attractor position (hide visualization)
            attractor_msg = Point()
            attractor_msg.x = float('inf')  # invalid position
            attractor_msg.y = float('inf')
            attractor_msg.z = float('inf')
            self.attractor_pos_pub.publish(attractor_msg)
            
            intent_msg = String()
            intent_msg.data = f"STOP: User preferred direction ({['Left', 'Forward', 'Right'][self.user_direction]}) not available, waiting for new command"
            self.wheelchair_intent_pub.publish(intent_msg)
            return
        
        # 2. check visibility and safety of user selected direction attractor position
        attractor_valid = self.check_attractor_visibility(attractor_pos, ranges, angle_min, angle_increment)
        
        if not attractor_valid:
            # if attractor cannot be safely placed, stop motion
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.path_pub.publish(cmd)
            
            intent_msg = String()
            intent_msg.data = "STOP: Unable to place attractor safely"
            self.wheelchair_intent_pub.publish(intent_msg)
            return
        
        # 3. update current attractor position (user selected direction)
        self.current_attractor_pos = attractor_pos
        
        # publish user selected attractor position to Unity visualization (coordinate system conversion in advance)
        attractor_msg = Point()

        unity_scale_factor = 5  # Unity will have a 0.2 scale factor so multiply by 5 here to ensure Attractor position is correct

        # ROS coordinate system -> Unity coordinate system conversion
        # ROS: X=forward, Y=left -> Unity: X=right, Y=height, Z=forward
        attractor_msg.x = float(attractor_pos['y'] * unity_scale_factor)  # ROS_Y -> Unity_X
        attractor_msg.y = 0.1 * unity_scale_factor  # Unity height fixed
        attractor_msg.z = float(attractor_pos['x'] * unity_scale_factor)  # ROS_X -> Unity_Z
        self.attractor_pos_pub.publish(attractor_msg)
        
        # 4. calculate attractive force (pointing to dynamic attractor)
        att_force_x = self.attractive_coef * attractor_pos['x']
        att_force_y = self.attractive_coef * attractor_pos['y']
        
        # 5. calculate repulsive force (from all obstacles)
        rep_force_x = 0.0
        rep_force_y = 0.0
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist) or dist > self.obstacle_influence:
                continue
                
            angle = angle_min + i * angle_increment
            
            # calculationrepulsive force
            repulsive_magnitude = self.repulsive_coef * (1.0 / dist - 1.0 / self.obstacle_influence)
            
            # obstacle position in laser coordinate system center
            obstacle_x = dist * math.cos(angle)
            obstacle_y = dist * math.sin(angle)
            
            # unit vector from obstacle to wheelchair (repulsive force direction)
            if obstacle_x == 0 and obstacle_y == 0:
                continue
                
            norm = math.sqrt(obstacle_x**2 + obstacle_y**2)
            rep_force_x += repulsive_magnitude * (-obstacle_x / norm)
            rep_force_y += repulsive_magnitude * (-obstacle_y / norm)
        
        # 6. combine attractive force and repulsive force
        total_force_x = att_force_x + rep_force_x
        total_force_y = att_force_y + rep_force_y
        
        # 7. check if need to stop (based on potential field strength, multipath handled by fusion node)
        total_force_magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
        
        should_stop = False
        stop_reason = ""
        
        if total_force_magnitude < 0.05:
            should_stop = True
            stop_reason = "Attractive force too weak"

        # 8. generate motion command
        cmd = Twist()
        
        if should_stop:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            wheelchair_intent = f"STOP: {stop_reason}"
        else:
            # convert potential field force to motion command
            # forward velocity: based on X direction force, limited within reasonable range
            cmd.linear.x = max(0.0, min(0.5, total_force_x))
            
            # steering velocity: based on Y direction force
            cmd.angular.z = max(-0.4, min(0.4, total_force_y))
            
            # adjust velocity based on distance to obstacles
            min_obstacle_dist = float('inf')
            for dist in ranges:
                if math.isfinite(dist):
                    min_obstacle_dist = min(min_obstacle_dist, dist)
            
            if min_obstacle_dist < 1.0:
                # reduce velocity when approaching obstacles
                speed_factor = min_obstacle_dist / 1.0
                cmd.linear.x *= speed_factor
                cmd.angular.z *= 1.1  # enhance steering capability
            
            # generate intent description
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
        
        # 9. publish command and intent
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
