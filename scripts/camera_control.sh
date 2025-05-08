#!/bin/zsh

# 检查/dart_launcher_detector是否存在


if [ "$1" == "open" ]; then
    ros2 lifecycle set /dart_launcher_detector configure && ros2 lifecycle set /dart_launcher_detector activate
    echo "Camera opened."
elif [ "$1" == "close" ]; then
    ros2 lifecycle set /dart_launcher_detector deactivate && ros2 lifecycle set /dart_launcher_detector cleanup
    echo "Camera closed."
else
    echo "Usage: $0 {open|close}"
    exit 1
fi