from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 ros_tcp_endpoint server
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_server',
            output='screen',
            parameters=[{'ROS_IP': '127.0.0.1'}],
        ),

        # 启动 shared control node
        Node(
            package='shared_control',
            executable='simple_shared_control',
            name='shared_control_node',
            output='screen'
        ),
    ])
