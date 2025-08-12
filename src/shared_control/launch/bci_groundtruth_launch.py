#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ground truth intent input node
        Node(
            package='shared_control',
            executable='groundtruth_input_node',
            name='groundtruth_input_node',
            output='screen',
            emulate_tty=True
        ),
        
        # BCI input node
        Node(
            package='shared_control',
            executable='bci_input_node',
            name='bci_input_node',
            output='screen'
        ),
        
        # Path evaluation node
        Node(
            package='shared_control',
            executable='path_eval_node',
            name='path_eval_node',
            output='screen'
        ),
        
        # Potential field planner node
        Node(
            package='shared_control',
            executable='potential_field_planner',
            name='potential_field_planner',
            output='screen'
        ),
        
        # Control fusion node (with ground truth waiting)
        Node(
            package='shared_control',
            executable='control_fusion_node',
            name='control_fusion_node',
            output='screen'
        ),
    ])
