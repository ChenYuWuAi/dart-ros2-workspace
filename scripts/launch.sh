#!/bin/zsh
export ROS_LOG_DIR=~/dart-ros2-workspace/launch_log
source ~/.zshrc
source ~/dart-ros2-workspace/install/setup.zsh
ros2 launch dart_launcher dart.launch.py
# 打印环境变量
printenv | grep ROS
