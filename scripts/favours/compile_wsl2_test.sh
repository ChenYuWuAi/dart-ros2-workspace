#!/bin/bash

export DART_DEVICE_TYPE="dart_launcher_wsl2"

export BUILD_TEST="true"

colcon build --merge-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_DART_ROS2_WORKSPACE_TESTS=ON \
    -G Ninja \
    --event-handlers console_direct+ \
    --packages-select \
    dart_msgs \
    cv_bridge dart_detector dart_test
