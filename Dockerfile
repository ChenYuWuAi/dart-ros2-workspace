# 本Dockerfile用于更新依赖环境。

FROM chenyuwuai/ros2_crosscompile:arm64-x86_64-20250401

RUN apt-get update && apt upgrade -y && apt autoremove -y
# RUN apt-get install -y ros-jazzy-lifecycle ros-jazzy-xacro

# COPY ./src/ /home/ros2_ws/src/

# RUN cd /home/ros2_ws/src \
#    && rosdep update --rosdistro ${ROS_DISTRO}  

# RUN rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} \
#    --skip-keys ros-${ROS_DISTRO}-joint-state-publisher-gui --skip-keys ros-${ROS_DISTRO}-rviz2\
#    && \
#    : "remove cache" && \
#    apt-get autoremove -y -qq && \
#    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-c"]