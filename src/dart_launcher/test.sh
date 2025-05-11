#!/bin/bash

# Define color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

kill_nodes() {
    pkill -f mock_dartmcu_node
    pkill -f node_dart_param_gateway
    # pkill -f "ros2 bag record"
    sleep 2
}

# Kill any existing processes
kill_nodes

# Source the ROS 2 workspace setup script
source ~/dart-ros2-workspace/install/setup.bash

# Define configuration path
CONFIG_PATH=~/dart-ros2-workspace/src/dart_launcher/config/

# 1. Start nodes in the background
echo -e "${BLUE}Starting nodes...${NC}"
ros2 run dart_test mock_dartmcu_node &
MOCK_NODE_PID=$!

ros2 run dart_launcher node_dart_param_gateway --ros-args --params-file $CONFIG_PATH/node_dart_param_gateway.yaml &
GATEWAY_NODE_PID=$!

# Allow nodes to initialize
sleep 5

# 2. Transition the lifecycle of the gateway node
echo -e "${BLUE}Configuring the gateway node...${NC}"
ros2 lifecycle set /node_dart_param_gateway configure
if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to configure the gateway node${NC}"
    kill_nodes
    exit 1
fi

ros2 lifecycle set /node_dart_param_gateway activate
if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to activate the gateway node${NC}"
    kill_nodes
    exit 1
fi

sleep 2

# # 启动ros2 bag录制
# ros2 bag record -o ~/dart-ros2-workspace/src/dart_launcher/test_bag \
#     /dart_launcher_detector/results/qrcode \
#     /dart_launcher_mcu/status \
#     /dart_launcher_mcu/cmd_params \
#     /dart_launcher_mcu/cmd_protocols
# BAG_PID=$!
# sleep 2

# 3. Check if parameter files are generated
echo -e "${BLUE}Checking parameter files...${NC}"
if [ -f $CONFIG_PATH/dart_param.json ] && [ -f $CONFIG_PATH/dart_protocols.json ]; then
    echo -e "${GREEN}Parameter files generated successfully${NC}"
else
    echo -e "${RED}Failed to generate parameter files${NC}"
    kill_nodes
    exit 1
fi

# 原有用例（为了完整性再贴一次）
declare -a qr_cases=(
    '{\"command_type\":\"DartParams\",\"data\":{\"primary_yaw\":543}}'
    '{\"command_type\":\"DartParams\",\"data\":{\"primary_yaw\":123,\"primary_force\":8888}}'
    '{\"command_type\":\"DartParams\",\"data\":{\"auto_aim_enabled\":true}}'
    '{\"command_type\":\"DartProtocols\",\"data\":{\"primary_force_offset\":999}}'

    # 新增用例：
    # 1. 只设置 auxiliary_yaw_offsets 数组
    '{\"command_type\":\"DartParams\",\"data\":{\"auxiliary_yaw_offsets\":[10,20,30,40]}}'
    # 2. 只设置 auxiliary_force_offsets 数组
    '{\"command_type\":\"DartParams\",\"data\":{\"auxiliary_force_offsets\":[5,15,25,35]}}'
    # 3. 同时设置 dart_launch_process_offset_begin 与 dart_launch_process_offset_end
    '{\"command_type\":\"DartParams\",\"data\":{\"dart_launch_process_offset_begin\":100,\"dart_launch_process_offset_end\":200}}'
    # 4. 设置 target_auto_aim_x_axis 浮点数
    '{\"command_type\":\"DartParams\",\"data\":{\"target_auto_aim_x_axis\":0.75}}'
    # 6. 全字段一次性更新（混合多种类型）
    '{\"command_type\":\"DartParams\",\"data\":{\"primary_yaw\":42,\"primary_force\":1500,\"primary_force_offset\":50,\"auxiliary_yaw_offsets\":[1,2,3,4],\"auxiliary_force_offsets\":[3,4,5,6],\"dart_launch_process_offset_begin\":20,\"dart_launch_process_offset_end\":80,\"auto_aim_enabled\":false,\"target_auto_aim_x_axis\":0.33}}'
)

declare -a grep_patterns=(
    543
    8888
    true
    999

    # 新增用例对应的校验关键字
    20            # auxiliary_yaw_offsets 中的任意一个值
    15            # auxiliary_force_offsets 中的任意一个值
    100           # launch_process_offset_begin
    0.75          # target_auto_aim_x_axis
    1500          # 全字段用例中 primary_force 的校验
)

# 测试循环（无改动）
for i in "${!qr_cases[@]}"; do
    echo -e "${YELLOW}Testing QR code case $((i + 1))...${NC}"
    ros2 topic pub --once /dart_launcher_detector/results/qrcode std_msgs/msg/String \
        "{\"data\": \"${qr_cases[$i]}\"}"
    if [ $? -ne 0 ]; then
        echo -e "${RED}Failed to publish QR code parameter message${NC}"
        kill_nodes
        exit 1
    fi
    sleep 2

    # 协议数据仍写到 dart_protocols.json
    if [ $i -eq 3 ]; then
        cfg_file="$CONFIG_PATH/dart_protocols.json"
    else
        cfg_file="$CONFIG_PATH/dart_param.json"
    fi

    # 校验写入
    grep "${grep_patterns[$i]}" "$cfg_file" >/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Case $((i + 1)) check passed!${NC}"
    else
        echo -e "${RED}Case $((i + 1)) check FAILED!${NC}"
        kill_nodes
        exit 1
    fi
    sleep 1

    # MCU 重启同步测试只在第 1 个用例后执行
    if [ $i -eq 0 ]; then
        echo -e "${BLUE}Simulating MCU restart...${NC}"
        pkill -f mock_dartmcu_node
        sleep 4
        ros2 run dart_test mock_dartmcu_node &>/dev/null &
        sleep 2
        echo -e "${BLUE}Verifying sync...${NC}"
        ros2 topic echo /dart_launcher_mcu/status --once | grep "last_param_update_time"
        sleep 2
    fi
done

echo -e "${BLUE}Killing nodes...${NC}"
kill_nodes
echo -e "${GREEN}All tests completed successfully${NC}"
exit 0
