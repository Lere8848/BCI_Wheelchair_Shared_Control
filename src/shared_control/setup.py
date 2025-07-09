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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_tcp_server = ros_tcp_endpoint.default_server_endpoint:main',  # entry point for the ros_tcp_endpoint server
            'user_input_node = shared_control.user_input_node:main',  # entry point for the user input node
            'path_eval_node = shared_control.path_eval_node:main',  # entry point for the path evaluation node
            'manual = shared_control.manual_control:main',  # entry point for the manual control node
            'demo = shared_control.simple_shared_control_demo:main', # entry point for the shared control node
            'static = shared_control.static_shared_control_fusion_node:main',  # entry point for the fusion node
        ],
    },
)
