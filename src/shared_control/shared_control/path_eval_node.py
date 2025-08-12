# path_eval_node.py
# Path evaluation node: Process laser scanner data from Unity simulation environment
#
# Important notes:
# - Laser scanner data source: Unity simulation environment
# - Coordinate system conversion: Unity coordinate system -> ROS coordinate system
# - Direction definition: Need to correctly map left-right directions according to Unity's coordinate system
#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Int8MultiArray, Bool
import numpy as np
import math

# LSL library import - for sending path information to BCI
try:
    from pylsl import StreamInfo, StreamOutlet
    LSL_AVAILABLE = True
except ImportError:
    LSL_AVAILABLE = False

class PathEvalNode(Node):
    def __init__(self):
        super().__init__('path_eval_node')
        
        # publisher subscription
        self.pub = self.create_publisher(Int8MultiArray, '/path_options', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.danger_pub = self.create_publisher(Bool, '/danger_stop', 10)
        
        # added publisher topic
        self.path_blocked_pub = self.create_publisher(Bool, '/path_blocked', 10)
        self.multipath_pub = self.create_publisher(Int8MultiArray, '/multipath_detected', 10)
        
        # data storage
        self.lidar_ranges = []
        self.laser_data = None
        
        # sliding window obstacle detection parameters
        self.window_size = 5
        self.min_consecutive_detections = 2
        self.obstacle_detection_history = []
        
        # detectionparameters
        self.path_detection_distance = 2.0 # pathdetectiondistance(m)
        self.path_width_threshold = 1.0 # path width threshold (m)
        self.wall_detection_distance = 0.5 # wall detection distance (m)
        self.min_obstacle_dist = 0.5 # minimum obstacle distance (m)

        # LSL输出流设置 - send path information to BCI
        self.lsl_outlet = None
        if LSL_AVAILABLE:
            try:
                # Create LSL stream: 3 channels [Left, Forward, Right]
                info = StreamInfo('Path_Info', 'Path_Info', 3, 5, 'float32', 'path_eval_node')
                self.lsl_outlet = StreamOutlet(info)
                self.get_logger().info('LSL outlet created for path info transmission to BCI')
            except Exception as e:
                self.get_logger().warn(f'Failed to create LSL outlet: {str(e)}')
        else:
            self.get_logger().warn('LSL not available - path info will not be sent to BCI')

        self.timer = self.create_timer(0.2, self.timer_callback)  # increase frequency
        self.get_logger().info('PathEvalNode initialized - Processing LIDAR data from Unity simulation environment')

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)
        self.laser_data = msg  # save complete laser data
        # self.get_logger().info(f'LIDAR data received: {len(self.lidar_ranges)} ranges')
    
    # def ultrasonic_callback(self, msg):
    #     self.front_ultrasonic = msg.range

    def analyze_sectors_openness(self, ranges, angle_min, angle_increment):
        """
        analyze openness of three major sectors (advanced method ported from potential_field_planner)
        return: dict {direction: {sub-sector openness list}}
        """
        # define three main sectors: Unity coordinate system consistent with potential_field_planner
        major_sectors = {
            'left': (-math.pi/2, -math.pi/6),    # leftsector：-90°到-30°
            'front': (-math.pi/6, math.pi/6),    # forwardsector：-30°到30°
            'right': (math.pi/6, math.pi/2)      # rightsector：30°到90°
        }
        
        sub_sectors_per_major = 6  # each main sector divided into 6 sub-sectors
        sector_analysis = {}
        
        for direction, (start_angle, end_angle) in major_sectors.items():
            sector_width = (end_angle - start_angle) / sub_sectors_per_major
            sub_sector_openness = []
            
            # analyze each sub-sector
            for i in range(sub_sectors_per_major):
                sub_start = start_angle + i * sector_width
                sub_end = start_angle + (i + 1) * sector_width
                sub_center = (sub_start + sub_end) / 2
                
                # calculate openness of this sub-sector
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
        calculate sub-sector openness (advanced method ported from potential_field_planner)
        """
        distances_in_sector = []
        
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
                
            angle = angle_min + i * angle_increment
            
            # check if within current sub-sector range
            if start_angle <= angle <= end_angle:
                distances_in_sector.append(dist)
        
        if len(distances_in_sector) == 0:
            return 0.0  # return impassable when no data
        
        distances = np.array(distances_in_sector)
        
        # opennesscalculation
        avg_distance = np.mean(distances)
        min_distance = np.min(distances)
        
        # basic openness: based on average distance
        base_openness = min(1.0, avg_distance / 3.0)
        
        # safety penalty: if minimum distance too small, greatly reduce openness
        if min_distance < 0.7:
            safety_penalty = 0.8  # greatly reduce
        elif min_distance < 1.3:
            safety_penalty = 0.5  # moderate reduction
        else:
            safety_penalty = 0.0  # no penalty
        
        final_openness = base_openness * (1.0 - safety_penalty)
        return max(0.0, min(1.0, final_openness))

    def detect_multiple_paths_vector(self, ranges, angle_min, angle_increment):
        """
        multipath detection based on advanced sector analysis (loosely coupled implementation using potential_field method)
        """
        # use advanced sector analysis method
        sector_analysis = self.analyze_sectors_openness(ranges, angle_min, angle_increment)
        
        # convert to simple passage status for compatibility with existing interface
        paths_status = []
        sector_order = ['left', 'front', 'right']
        
        for direction in sector_order:
            if direction in sector_analysis:
                # calculate average openness of this main sector
                openness_values = [sub['openness'] for sub in sector_analysis[direction]]
                avg_openness = np.mean(openness_values) if openness_values else 0.0
                max_openness = np.max(openness_values) if openness_values else 0.0
                
                # passage judgment: average openness > 0.3 and maximum openness > 0.4
                is_clear = avg_openness > 0.3 and max_openness > 0.4
                paths_status.append(is_clear)
                
                self.get_logger().debug(
                    f"{direction.upper()} sector: avg_openness={avg_openness:.3f}, "
                    f"max_openness={max_openness:.3f}, clear={is_clear}"
                )
            else:
                paths_status.append(False)
        
        return paths_status

    def timer_callback(self):
        if len(self.lidar_ranges) == 0 or self.laser_data is None:
            return

        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        # === use unified advanced vector method for path analysis ===
        paths_status = self.detect_multiple_paths_vector(ranges, angle_min, angle_increment)
        open_paths_count = sum(paths_status)
        
        # use vector analysis results as path_options to ensure consistency
        path = [int(p) for p in paths_status]  # [left, forward, right] consistent with vector analysis results
        
        # check forward obstacles (for sliding window validation)
        immediate_front_blocked = False
        for i, dist in enumerate(ranges):
            if not math.isfinite(dist):
                continue
            angle = angle_min + i * angle_increment
            # check if within forward ±17 degrees range
            if abs(angle) < 0.3 and dist < self.min_obstacle_dist:
                immediate_front_blocked = True
                break
        
        # multipath detection logic
        multipath_detected = False
        if open_paths_count >= 2:  # at least two paths are open
            multipath_detected = True
            self.get_logger().info(f'Multiple paths detected! Open paths: Left={paths_status[0]}, Front={paths_status[1]}, Right={paths_status[2]}')
        
        # comprehensive judgment on whether to block path - based on vector analysis results
        path_blocked = False
        block_reason = ""

        # danger detection - reduce sensitivity to avoid excessive interference
        all_dists = np.array(self.lidar_ranges)
        # change to stricter conditions: 0.4m distance and must be within smaller angle range forward
        danger = False
        
        for i, dist in enumerate(all_dists):
            if not math.isfinite(dist):
                continue
            # calculate angle (simplified calculation, 180-degree scan)
            angle = (i / len(all_dists)) * 2 * math.pi - math.pi

            # only trigger danger if within forward ±20 degrees range and distance < 0.4m
            if abs(angle) < math.pi/9 and dist < 0.4:  # ±20，0.4m
                danger = True
                self.get_logger().warn(f'Danger detected: obstacle at {dist:.2f}m in front')
                break

        # === publish all messages ===
        path_msg = Int8MultiArray()
        path_msg.data = path
        self.pub.publish(path_msg)

        # publish vector analysis results as multipath detection (now same as path_options)
        multipath_msg = Int8MultiArray()
        multipath_msg.data = path  # now consistent with path_options
        self.multipath_pub.publish(multipath_msg)

        # send path information to BCI system [Left, Forward, Right]
        if self.lsl_outlet and LSL_AVAILABLE:
            try:
                path_info = [float(path[0]), float(path[1]), float(path[2])]
                self.lsl_outlet.push_sample(path_info)
            except Exception as e:
                self.get_logger().warn(f'Failed to send path info via LSL: {str(e)}')

        # publish path blocked status
        blocked_msg = Bool()
        blocked_msg.data = path_blocked
        self.path_blocked_pub.publish(blocked_msg)

        # publish danger status
        danger_msg = Bool()
        danger_msg.data = bool(danger)
        self.danger_pub.publish(danger_msg)

        # log output
        self.get_logger().info(f'/path_options: {path} | /multipath: {path} | /path_blocked: {path_blocked} ({block_reason}) | /danger_stop: {danger}')
        
def main(args=None):
    rclpy.init(args=args)
    node = PathEvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
