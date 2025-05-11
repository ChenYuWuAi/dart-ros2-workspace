#!/bin/zsh
# 通过给生命周期控制节点发送std_srvs/Trigger服务来停止所有节点
source ~/.zshrc
ros2 service call /shutdown_all std_srvs/srv/Trigger
# 终止生命周期节点
pkill -f "dart-ros2-workspace/install" && pkill -f "dart.launch.py"
