#!/usr/bin/env python3
"""
Ground Truth Input Node - read user keyboard input and transmit true intent to BCI system
keyboard mapping: W=forward(1), A=left turn(0), D=right turn(2)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys
import platform

# LSL imports
try:
    from pylsl import StreamInfo, StreamOutlet
    LSL_AVAILABLE = True
except ImportError:
    LSL_AVAILABLE = False

if platform.system() == 'Windows':
    import msvcrt
else:
    import termios
    import tty
    import select

class GroundTruthInputNode(Node):
    def __init__(self):
        super().__init__('groundtruth_input_node')
        
        # ROS2 publisher
        self.pub = self.create_publisher(Int8, '/gt_input', 10)
        
        # LSL output stream setup - send user true intent to BCI
        self.lsl_outlet = None
        if LSL_AVAILABLE:
            try:
                # Create LSL stream: 1 channel to send true intent [0=Left, 1=Right, 2=Forward]
                info = StreamInfo('GroundTruth_Intent', 'Intent', 1, 0, 'float32', 'groundtruth_input_node')
                self.lsl_outlet = StreamOutlet(info)
                self.get_logger().info('LSL outlet created for ground truth intent transmission to BCI')
            except Exception as e:
                self.get_logger().warn(f'Failed to create LSL outlet: {str(e)}')
        else:
            self.get_logger().warn('LSL not available - ground truth intent will not be sent to BCI')
        
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Press a/w/d for Ground Truth: Left/Forward/Right')

    def get_key(self):
        if platform.system() == 'Windows':
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8')
            else:
                return ''
        else:
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            key = sys.stdin.read(1) if rlist else ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

    def send_groundtruth_intent(self, intent):
        """send user true intent to BCI system"""
        if self.lsl_outlet and LSL_AVAILABLE:
            try:
                intent_data = [float(intent)]
                self.lsl_outlet.push_sample(intent_data)
            except Exception as e:
                self.get_logger().warn(f'Failed to send ground truth intent via LSL: {str(e)}')

    def timer_callback(self):
        key = self.get_key()
        msg = Int8()

        if key == 'a':
            msg.data = 0  # left
            self.pub.publish(msg)
            self.send_groundtruth_intent(0)
            self.get_logger().info('Ground Truth: LEFT')
        elif key == 'w':
            msg.data = 2  # forward
            self.pub.publish(msg)
            self.send_groundtruth_intent(2)
            self.get_logger().info('Ground Truth: FORWARD')
        elif key == 'd':
            msg.data = 1  # right
            self.pub.publish(msg)
            self.send_groundtruth_intent(1)
            self.get_logger().info('Ground Truth: RIGHT')


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
