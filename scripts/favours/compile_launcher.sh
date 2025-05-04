#!/bin/bash

export DART_DEVICE_TYPE="dart_launcher"

colcon build --merge-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -G Ninja \
    --event-handlers console_direct+ \
    --packages-select \
    dart_msgs dart_launcher \
    cv_bridge dart_detector
