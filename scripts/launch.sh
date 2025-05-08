#!/bin/zsh
export ROS_LOG_DIR=/home/chenyu/dart24_ws/launch_log
source ~/.zshrc
source ~/dart_ros2_workspace/install/setup.zsh
# 打印环境变量
printenv | grep ROS
