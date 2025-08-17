# BCI Wheelchair Shared Control System

This repository contains a ROS2-based shared control system for BCI (Brain-Computer Interface) controlled wheelchairs, integrating potential field path planning.

## Related Projects

**This project works with the following ROS2 project**
- [BCI_Wheelchair_Simulator](https://github.com/Lere8848/BCI_Wheelchair_Simulator) - BCI wheelchair simulator in Unity.

Both projects need to run simultaneously, with the Unity simulator serving as the hardware platform and the ROS2 project providing control algorithms.

## Prerequisites

### Required Software
- **ROS2 Humble** (recommended)
- **Python 3.8+**
- **Conda/Miniconda** for environment management
- **Unity 6000.0.40f1** (for simulation environment)

### Required Python Packages
```bash
pylsl  # Lab Streaming Layer for BCI communication
numpy
```

## Installation and Setup

### 1. Environment Setup

(**For ROS2 installation in Windows**, RoboStack is reccomended, please refer to: [RoboStack Getting Started Guide](https://robostack.github.io/GettingStarted.html))

After ROS2 installation, activate your Conda environment:
```bash
conda activate ros_env
```

Install required packages:
```bash
pip install pylsl numpy
```
### 2. Unity-ROS2 Connection Setup

This system uses the **ROS-TCP-Endpoint** package for Unity-ROS2 communication:
- ROS2 Package: [ROS-TCP-Endpoint (main-ros2 branch)](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2)
- Detailed Connection Tutorial: [Unity-ROS Integration Setup Guide](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md)

<!-- **Unity Simulator Repository**: https://github.com/Lere8848/BCI_Wheelchair_Simulator -->

If you want to connect directly to Unity without using the launch file:

```bash
# Start ROS-TCP-Endpoint for Unity communication
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

**Unity Configuration:**
1. Install the ROS TCP Connector package in Unity via `Package Manager`
2. Navigate to **Robotics > ROS Settings** in Unity menu
3. Set ROS IP to `127.0.0.1` and Port to `10000`


### 3. ROS2 Workspace Setup

Clone this repository to your ROS2 workspace and build the package:
```bash
# Build the shared_control package
colcon build --packages-select shared_control 

# Setup the environment (Windows)
call install/setup.bat

# Setup the environment (Linux/Mac)
source install/setup.bash
```

## Usage

### Launch the Complete System

Activate the environment and launch all nodes:
```bash
# Activate conda environment
conda activate ros_env

# Setup ROS2 environment
call install/setup.bat  # Windows
# source install/setup.bash  # Linux/Mac

# Launch the integrated BCI system
ros2 launch shared_control bci_integration_launch.py
```

### Launch Individual Nodes (Alternative)

For debugging or custom configurations, you can launch nodes individually:
```bash
# Set environment variable for immediate log output
set PYTHONUNBUFFERED=1  # Windows
# export PYTHONUNBUFFERED=1  # Linux/Mac

# Launch individual nodes in separate terminals
ros2 run shared_control bci_input_node
ros2 run shared_control potential_field_planner  
ros2 run shared_control path_eval_node
ros2 run shared_control control_fusion_node
```

## System Architecture

### Core Nodes:
The system consists of four main nodes:

1. **bci_input_node**: Receives BCI signals via LSL (Lab Streaming Layer)
2. **path_eval_node**: Processes LIDAR data for path evaluation and obstacle detection
3. **potential_field_planner**: Generates motion commands using potential field algorithms
4. **control_fusion_node**: Fuses user intent with autonomous navigation for shared control

### Key Topics:
- `/user_cmd`: User intent commands (Int8)
- `/cmd_vel`: Final velocity commands (Twist)
- `/scan`: LIDAR sensor data (LaserScan)
- `/path_options`: Available path directions (Int8MultiArray)
- `/bci_info`: BCI confidence information (Float32MultiArray)