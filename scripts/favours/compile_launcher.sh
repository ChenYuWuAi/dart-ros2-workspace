#!/bin/bash

export DART_DEVICE_TYPE="dart_launcher"

echo -e "\033[1;32mBuilding ROS2 packages for $DART_DEVICE_TYPE\033[0m"

echo -e "\033[1;33mStopping dart_ros2_run.service\033[0m"
sudo systemctl stop dart_ros2_run.service
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
