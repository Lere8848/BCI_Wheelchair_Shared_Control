# same as semi auto one, but the input is integrated with BCI motion imagery.
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

        # Potential field path planning node
        Node(
            package='shared_control',
            executable='potential_field_planner',
            name='potential_field_planner'
        ),

        # Fusion node
        Node(
            package='shared_control',
            executable='control_fusion_node',
            name='control_fusion_node',
        ),

        # BCI input node (replaces original user_input_node)
        Node(
            package='shared_control',
            executable='bci_input_node',
            name='bci_input_node'
        ),

        # Path evaluation node
        Node(
            package='shared_control',
            executable='path_eval_node',
            name='path_eval_node'
        )
    ])
