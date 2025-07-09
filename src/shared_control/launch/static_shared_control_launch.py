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

        # Fusion node
        Node(
            package='shared_control',
            executable='static',
            name='static_shared_control_fusion_node',
        ),

        # User input node
        Node(
            package='shared_control',
            executable='user_input_node',
            name='user_input_node'
        ),

        # Path evaluation node
        Node(
            package='shared_control',
            executable='path_eval_node',
            name='path_eval_node'
        )
    ])
