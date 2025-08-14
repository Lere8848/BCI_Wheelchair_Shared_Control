# bci_input_node.py
# Note: BCI side must send data first before this node can receive it. Due to time constraints, 
# this will not be investigated further. Will optimize this later when time permits.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32MultiArray
import numpy as np
import time
import threading

# LSL library import - for receiving BCI data
try:
    from pylsl import StreamInlet, resolve_streams
    LSL_AVAILABLE = True
except ImportError:
    LSL_AVAILABLE = False

class BCIInputNode(Node):
    def __init__(self):
        super().__init__('bci_input_node')
        
        self.user_cmd_pub = self.create_publisher(Int8, '/user_cmd', 10)  # use the original user_input_node topic
        self.bci_info_pub = self.create_publisher(Float32MultiArray, '/bci_info', 10)  # publish BCI info to Unity
        
        # BCI Confidence Data
        self.latest_confidences = [0.0, 0.0, 0.0]  # Confidences for three directions [left, right, forward]

        # BCI Validation Data
        self.latest_action = 2      # action: actual direction to go (0=left, 1=right, 2=forward)
        self.latest_valid = 0       # if valid: whether command should be executed (1=execute, 0=no execute)

        # LSL data stream related
        self.confidence_inlet = None  # Confidence data stream
        self.validation_inlet = None  # Validation data stream
        self.lsl_connected = False
        
        # Timer - periodically publish BCI info to Unity
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Start LSL receiver thread
        if LSL_AVAILABLE:
            self.lsl_thread = threading.Thread(target=self.lsl_receiver_thread, daemon=True)
            self.lsl_thread.start()
            self.get_logger().info('BCI Input Node initialized with LSL support')
        else:
            self.get_logger().warn('LSL library not available - BCI node cannot function without LSL')
            # In case LSL is not available, publish default data to keep system running
            self.latest_confidences = [0.0, 0.0, 0.0]  # Zero confidence
            self.latest_action = 2  # Default forward
            self.latest_valid = 0  # No execution

    def lsl_receiver_thread(self):
        """LSL data receiver thread - handle two separate streams"""
        try:
            # Look for BCI data streams
            self.get_logger().info('Looking for BCI data streams...')
            streams = resolve_streams()
            
            # Find name: ClassifierConfidenceData, type: Confidence
            confidence_streams = [s for s in streams if s.type() == 'Confidence']
            # Find name: ClassifierValidationData, type: Validation
            validation_streams = [s for s in streams if s.type() == 'Validation']
            
            if not confidence_streams:
                self.get_logger().warn('No Confidence data stream found, waiting for stream...')
                time.sleep(1.0)
                return
                
            if not validation_streams:
                self.get_logger().warn('No Validation data stream found, waiting for stream...')
                time.sleep(1.0)
                return
            
            # Connect to streams
            self.confidence_inlet = StreamInlet(confidence_streams[0])
            self.validation_inlet = StreamInlet(validation_streams[0])
            self.lsl_connected = True
            self.get_logger().info('Connected to both Confidence and Validation streams')
            
            # Continuously receive data from both streams
            while rclpy.ok() and self.lsl_connected:
                try:
                    # Receive confidence data: [L, R, F]
                    conf_sample, conf_timestamp = self.confidence_inlet.pull_sample(timeout=0.01)
                    if conf_sample is not None and len(conf_sample) >= 3:
                        self.latest_confidences = [
                            float(conf_sample[0]),  # Left direction confidence
                            float(conf_sample[1]),  # Right direction confidence
                            float(conf_sample[2])   # Forward direction confidence
                        ]
                    
                    # Receive validation data: [action(0=L,1=R,2=F), if_valid(0/1)]
                    val_sample, val_timestamp = self.validation_inlet.pull_sample(timeout=0.01)
                    if val_sample is not None and len(val_sample) >= 2:
                        self.latest_action = int(val_sample[0])    # action
                        self.latest_valid = int(val_sample[1])     # if_valid
                        
                        # If valid is 1, publish user command
                        if self.latest_valid == 1:
                            self.publish_user_command()

                except Exception as e:
                    self.get_logger().warn(f'LSL data reception error: {str(e)}')
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f'LSL connection error: {str(e)}')
            self.lsl_connected = False

    def publish_user_command(self):
        """Publish user command to ROS system"""
        if self.latest_action in [0, 1, 2]:
            msg = Int8()
            msg.data = self.latest_action
            self.user_cmd_pub.publish(msg)
            
            direction_names = ['LEFT', 'RIGHT', 'FORWARD']
            selected_confidence = self.latest_confidences[self.latest_action]
            self.get_logger().info(
                f'BCI Intent: {direction_names[self.latest_action]} '
                f'(confidence: {selected_confidence:.3f})'
            )

    def timer_callback(self):
        """Periodically publish BCI info to Unity"""
        # Create message containing BCI status information
        # Send conf data format: [conf_left, conf_right, conf_forward]
        bci_info_msg = Float32MultiArray()
        bci_info_msg.data = [
            self.latest_confidences[0],     # Left direction confidence
            self.latest_confidences[1],     # Right direction confidence
            self.latest_confidences[2],     # Forward direction confidence
            0.6 # set by Yuze side, vis only (put in the LSL stream in future)
        ]
        self.bci_info_pub.publish(bci_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BCIInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
