# path_eval_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int8MultiArray  # 可用简单数组表示三个方向

class PathEvalNode(Node):
    def __init__(self):
        super().__init__('path_eval_node')
        self.pub = self.create_publisher(Int8MultiArray, '/path_options', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('PathEvalNode initialized. Publishing path options every second.')

    def timer_callback(self):
        # mock 可通方向：0=不可通，1=可通
        # 此时只允许向右或向左走（前边是死路）
        # 后面需要在这里加入实际的路径评估逻辑 
        mock_state = [0, 1, 1]  # 左=1 前=0 右=1

        msg = Int8MultiArray()
        msg.data = mock_state
        self.pub.publish(msg)

        self.get_logger().info(f'published path options: LEFT={mock_state[0]}, FORWARD={mock_state[1]}, RIGHT={mock_state[2]}')

def main(args=None):
    rclpy.init(args=args)
    node = PathEvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
