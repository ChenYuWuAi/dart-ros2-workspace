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
    pkill -f "ros2 bag record"
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
ros2 run dart_test mock_dartmcu_node & > /dev/null
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

# 启动ros2 bag录制
ros2 bag record -o ~/dart-ros2-workspace/src/dart_launcher/test_bag \
    /dart_launcher_detector/results/qrcode \
    /dart_launcher_mcu/status \
    /dart_launcher_mcu/cmd_params \
    /dart_launcher_mcu/cmd_protocols 
BAG_PID=$!
sleep 2

# 3. Check if parameter files are generated
echo -e "${BLUE}Checking parameter files...${NC}"
if [ -f $CONFIG_PATH/dart_param.json ] && [ -f $CONFIG_PATH/dart_protocols.json ]; then
    echo -e "${GREEN}Parameter files generated successfully${NC}"
else
    echo -e "${RED}Failed to generate parameter files${NC}"
    kill_nodes
    exit 1
fi

# 4. Define multiple QR code test cases
declare -a qr_cases=(
    '{\"command_type\":\"DartParams\",\"data\":{\"primary_yaw\":543}}'
    '{\"command_type\":\"DartParams\",\"data\":{\"primary_yaw\":123,\"primary_force\":8888}}'
    '{\"command_type\":\"DartParams\",\"data\":{\"primary_yaw_offset\":77}}'
    '{\"command_type\":\"DartProtocols\",\"data\":{\"primary_force_offset\":999}}'
)

declare -a grep_patterns=(
    543
    8888
    77
    999
)

# 5. Loop through test cases
for i in "${!qr_cases[@]}"; do
    echo -e "${YELLOW}Testing QR code case $((i+1))...${NC}"
    ros2 topic pub --once /dart_launcher_detector/results/qrcode std_msgs/msg/String \
         "{\"data\": \"${qr_cases[$i]}\"}"
    if [ $? -ne 0 ]; then
        echo -e "${RED}Failed to publish QR code parameter message${NC}"
        kill_nodes
        exit 1
    fi
    sleep 2
    # Check for expected value in the correct file
    if [ $i -eq 3 ]; then
        grep ${grep_patterns[$i]} $CONFIG_PATH/dart_protocols.json > /dev/null
    else
        grep ${grep_patterns[$i]} $CONFIG_PATH/dart_param.json > /dev/null
    fi
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}QR code parameters for case $((i+1)) successfully written and persisted${NC}"
    else
        echo -e "${RED}Failed to write QR code parameters for case $((i+1))${NC}"
        kill_nodes
        exit 1
    fi
    sleep 1
    
    # 6. 模拟MCU节点重启，last_param_update_time置0，检查gateway是否自动同步参数
    if [ $i -eq 0 ]; then
        echo -e "${BLUE}Simulating MCU node restart (last_param_update_time=0)...${NC}"
        pkill -f mock_dartmcu_node
        sleep 4
        # 重新启动mock_dartmcu_node
        ros2 run dart_test mock_dartmcu_node & > /dev/null
        sleep 2
        echo -e "${BLUE}Checking if parameters are synchronized...${NC}"
        # 检查参数是否被同步
        RESULT=$(ros2 topic echo /dart_launcher_mcu/status --once | grep "last_param_update_time")
        echo -e "${YELLOW}Result: $RESULT${NC}"
        sleep 2
    fi

done

# 6. Kill both nodes
echo -e "${BLUE}Killing nodes...${NC}"
kill_nodes
echo -e "${GREEN}All tests completed successfully${NC}"

exit 0