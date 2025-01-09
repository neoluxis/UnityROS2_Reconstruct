#!/bin/zsh

source /opt/ros/humble/setup.zsh
source /home/neolux/workspace/Unity/ROS_Pack/install/setup.zsh

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

echo "Exit"
