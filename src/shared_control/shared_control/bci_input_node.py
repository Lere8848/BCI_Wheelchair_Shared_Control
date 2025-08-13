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
        
        # Publisher setup
        self.user_cmd_pub = self.create_publisher(Int8, '/user_cmd', 10)  # Compatible with original user_input_node
        self.bci_info_pub = self.create_publisher(Float32MultiArray, '/bci_info', 10)  # New: publish BCI info to Unity
        
        # BCI data status
        self.latest_action = 1      # gt field: actual direction to go (0=left, 1=forward, 2=right)
        self.latest_confidences = [0.0, 0.0, 0.0]  # Confidences for three directions [left, right, forward]  MODIFY HERE LATER
        self.latest_valid = 0       # valid field: whether command should be executed (1=execute, 0=no execute)
        
        # LSL data stream related
        self.inlet = None
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
            self.latest_action = 1  # Default forward
            self.latest_confidences = [0.0, 0.0, 0.0]  # Zero confidence
            self.latest_valid = 0  # No execution

    def lsl_receiver_thread(self):
        """LSL data receiver thread"""
        try:
            # Look for BCI data stream
            self.get_logger().info('Looking for BCI data stream...')
            streams = resolve_streams()
            
            # Filter streams with BCI_Commands type
            bci_streams = [s for s in streams if s.type() == 'BCI_Commands'] # Interface with Yuze
            
            if not bci_streams:
                self.get_logger().warn('No BCI data stream found, waiting for stream...')
                # Wait and retry
                time.sleep(2.0)
                return
            
            # Connect to the first found stream
            self.inlet = StreamInlet(bci_streams[0])
            self.lsl_connected = True
            self.get_logger().info('Connected to BCI data stream')
            
            # Continuously receive data
            while rclpy.ok() and self.lsl_connected:
                try:
                    # Receive data format: dict{0:conf, 1:conf, 2:conf, gt:0/1/2, valid:1/0}
                    sample, timestamp = self.inlet.pull_sample(timeout=1.0)
                    
                    if sample is not None and len(sample) >= 5:
                        # Parse dictionary format data
                        self.latest_confidences = [
                            float(sample[0]),  # Left direction confidence
                            float(sample[1]),  # Right direction confidence
                            float(sample[2])   # Forward direction confidence
                        ]
                        self.latest_action = int(sample[3])    # gt field: actual direction
                        self.latest_valid = int(sample[4])     # valid field: whether to execute
                        
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
        # Send data format: [conf_left, conf_right, conf_forward, gt, valid, connected, threshold]
        bci_info_msg = Float32MultiArray()
        bci_info_msg.data = [
            self.latest_confidences[0],     # Left direction confidence
            self.latest_confidences[1],     # Right direction confidence
            self.latest_confidences[2],     # Forward direction confidence
            float(self.latest_action),      # gt: actual selected direction
            float(self.latest_valid),       # valid: whether to execute
            1.0 if self.lsl_connected else 0.0,  # Connection status
            1.0 # Confidence judgment threshold - placeholder for now, may be needed later
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
