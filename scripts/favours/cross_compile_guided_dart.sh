#/bin/bash

export DART_DEVICE_TYPE="dart_launcher"

colcon build --merge-install \
    --cmake-force-configure \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_TOOLCHAIN_FILE=$(pwd)/toolchain.cmake \
    -G Ninja \
    --event-handlers console_direct+ \
    --packages-select \
    cv_bridge dart_msgs dart_flysystem_description dart_flysystem_hardware dart_detector
