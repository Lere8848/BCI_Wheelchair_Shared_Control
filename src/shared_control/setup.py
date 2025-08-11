from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'shared_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), # add this line to include launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # add this line to include config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yukai Wang',
    maintainer_email='caroyalqi@gmail.com',
    description='a shared control package for BCI wheelchair',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ROS TCP Endpoint
            'ros_tcp_server = ros_tcp_endpoint.default_server_endpoint:main',
            # Input Nodes
            'user_input_node = shared_control.user_input_node:main',
            'bci_input_node = shared_control.bci_input_node:main',
            # Shared Control Nodes
            'path_eval_node = shared_control.path_eval_node:main',
            'potential_field_planner = shared_control.potential_field_planner:main',
            'control_fusion_node = shared_control.control_fusion_node:main',
            # For Test only
            'manual_control = shared_control.manual_control_node:main',
        ],
    },
)
