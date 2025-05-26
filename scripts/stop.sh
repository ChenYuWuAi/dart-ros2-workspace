#!/bin/zsh
# 通过向node_dart_launcher_lifecycle发送信号来触发关闭流程

# 获取node_dart_launcher_lifecycle的PID
LIFECYCLE_PID=$(ps aux | grep "node_dart_launcher_lifecycle" | grep -v grep | awk '{print $2}')

if [ -n "$LIFECYCLE_PID" ]; then
    echo "发送关闭信号给node_dart_launcher_lifecycle(PID: $LIFECYCLE_PID)"
    # 发送SIGUSR1信号
    kill -USR1 $LIFECYCLE_PID

    # 等待最多5秒，让关闭流程开始
    echo "等待关闭流程开始..."
    sleep 5
else
    echo "未找到node_dart_launcher_lifecycle进程"
fi

# 如果超时或未能正常关闭，强制终止所有相关进程
echo "强制终止所有剩余的ROS节点..."
pkill -f "dart-ros2-workspace/install" -9 && pkill -f "dart.launch.py"

exit 0