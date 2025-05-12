#!/bin/bash

export DART_DEVICE_TYPE="dart_launcher"

echo -e "\033[1;32mBuilding ROS2 packages for $DART_DEVICE_TYPE\033[0m"

SERVICE_RESTART_REQUIRED=0
# Check Service Status
if systemctl is-active --quiet dart_ros2_run.service; then
    echo -e "\033[1;32mService is running\033[0m"
    SERVICE_RESTART_REQUIRED=1
else
    echo -e "\033[1;31mService is not running\033[0m"
fi

if [ $SERVICE_RESTART_REQUIRED -eq 1 ]; then
    echo -e "\033[1;33mStopping dart_ros2_run.service\033[0m"
    sudo systemctl stop dart_ros2_run.service
fi
echo -e "\033[1;33mCompiling dart_ros2 packages\033[0m"

colcon build --merge-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -G Ninja \
    --event-handlers console_direct+ \
    --packages-select \
    dart_msgs dart_launcher \
    cv_bridge dart_detector

if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to build ROS2 packages\033[0m"
    exit 1
fi

echo -e "\033[1;32mSuccessfully built ROS2 packages\033[0m"

if [ $SERVICE_RESTART_REQUIRED -eq 1 ]; then
    echo -e "\033[1;33mStarting dart_ros2_run.service\033[0m"
    sudo systemctl start dart_ros2_run.service
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to start dart_ros2_run.service\033[0m"
        exit 1
    fi
    echo -e "\033[1;32mService restarted successfully\033[0m"
else
    echo -e "\033[1;32mNo need to restart the service\033[0m"
fi
