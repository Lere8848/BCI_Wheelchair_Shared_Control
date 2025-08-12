继续阅读shared control的文献 搞清楚 PSC 找到具体的baseline、具体要解决的问题（就是 我引入了errP之后，我要用什么方法去实现共享控制） 然后确定下evaluation的metrics

# TODO

- [x] 注意平台兼容性 当前 `src\shared_control\shared_control\simple_shared_control.py` 里面只支持了linux的键位 *(已解决 现支持lin+win)*
- [ ]

https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md 安装教程
https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md 总教程

1 Unity
    通过url git 安装了ROS TCP Connector这个package
    在顶部Robotics菜单的ROS settings中 设置同2中设定的ip和port gate


2 ROS2
    构建新的Workspace 在D:programming下
    clone并构建了ROS-TCP-Endpoint包（ros2 branch）
    两个都在本机 设置了127.0.0.1的ip 默认gate 10000
    
    往后每次启动，build+setup 然后run：
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1