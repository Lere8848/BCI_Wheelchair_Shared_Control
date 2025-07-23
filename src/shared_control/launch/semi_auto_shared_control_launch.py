from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROS-TCP-Endpoint
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            parameters=[{'ROS_IP': '127.0.0.1'}]
        ),

        # 势场法路径规划节点
        Node(
            package='shared_control',
            executable='potential_field_planner',
            name='potential_field_planner'
        ),

        # 融合节点
        Node(
            package='shared_control',
            executable='static',
            name='static_shared_control_fusion_node',
        ),

        # 用户输入节点
        Node(
            package='shared_control',
            executable='user_input_node',
            name='user_input_node'
        ),

        # 路径评估节点
        Node(
            package='shared_control',
            executable='path_eval_node',
            name='path_eval_node'
        ),

        # 无输入监视节点
        Node(
            package='shared_control',
            executable='no_input_monitor',
            name='no_input_monitor_node'
        )
    ])
