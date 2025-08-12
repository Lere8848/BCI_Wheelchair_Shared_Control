#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 地面真实意图输入节点
        Node(
            package='shared_control',
            executable='groundtruth_input_node',
            name='groundtruth_input_node',
            output='screen',
            emulate_tty=True
        ),
        
        # BCI输入节点
        Node(
            package='shared_control',
            executable='bci_input_node',
            name='bci_input_node',
            output='screen'
        ),
        
        # 路径评估节点
        Node(
            package='shared_control',
            executable='path_eval_node',
            name='path_eval_node',
            output='screen'
        ),
        
        # 势场规划器节点
        Node(
            package='shared_control',
            executable='potential_field_planner',
            name='potential_field_planner',
            output='screen'
        ),
        
        # 控制融合节点（带ground truth等待）
        Node(
            package='shared_control',
            executable='control_fusion_node',
            name='control_fusion_node',
            output='screen'
        ),
    ])
