/**
 * @file node_dart_param_gateway.hpp
 * @brief NodeDartParamGateway 类头文件
 */
#ifndef NODE_DART_PARAM_GATEWAY_HPP
#define NODE_DART_PARAM_GATEWAY_HPP

#include <thread>
#include <chrono>
#include <mutex> // 添加互斥锁

// ROS2 Lifecycle Node
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// Std_msg
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

// Dart_msg
#include <dart_msgs/msg/dart_launcher_params.hpp>
#include <dart_msgs/msg/dart_launcher_status.hpp>
#include <dart_msgs/msg/green_light.hpp>

// ROS2 Service
#include <std_srvs/srv/empty.hpp>

// Json
#include <nlohmann/json.hpp>

#include "dart_comm_share/include/dart_launcher_param.h"
#include "dart_mcu/Drivers/stm32-buzzer/src/buzzer_examples.h"
#include "dart_comm_share/include/dart_launcher_default_value.hpp"

using json = nlohmann::json;

#ifndef CONFIG_PATH
#define CONFIG_PATH "/home/chenyu/dart-ros2-workspace/install/dart_launcher/share/dart_launcher/config" + "dart_launcher_params.json"
#endif

// 自适应弹速控制结构体
struct AdaptiveVelocityControl {
    bool enabled = false;
    std::vector<double> expected_velocities = {0.0, 0.0, 0.0, 0.0}; // 预期速度值
    uint64_t last_launch_time = 0; // 上次发射时间
    double velocity_dead_zone = 0.3; // 速度误差死区
    double linear_coefficient = 10000.0; // 线性调整系数
};

class NodeDartParamGateway : public rclcpp_lifecycle::LifecycleNode
{
public:
    NodeDartParamGateway() = default;
    NodeDartParamGateway(rclcpp::NodeOptions options);
    ~NodeDartParamGateway();
    void load_and_save_default_value();
    bool load_params_from_file(std::string database_path);
    bool save_params_to_file(std::string database_path);
    void start_daemon();
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    void process_qr_code(const std_msgs::msg::String::SharedPtr msg);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

private:
    void daemon_thread_func();
    // 自适应弹速控制相关方法
    void update_adaptive_velocity_control();
    int32_t calculate_force_offset(double actual_velocity, double expected_velocity);
    void handle_dart_launch(uint8_t dart_launch_process, double actual_velocity = 0.0);
    
    // ROS2 Lifecycle Node
    rclcpp::Node::SharedPtr node_;

    // Publisher
    rclcpp::Publisher<dart_msgs::msg::DartLauncherParams>::SharedPtr dart_params_pub_,
        dart_protocols_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dart_buzzer_cmd_pub_;

    // Subscriber
    // for string json params
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dart_qr_param_sub_;
    // for status subscription
    rclcpp::Subscription<dart_msgs::msg::DartLauncherStatus>::SharedPtr dart_status_sub_;
    // for greenlight detector
    rclcpp::Subscription<dart_msgs::msg::GreenLight>::SharedPtr greenlight_sub_;

    // Dart_param (用于保存目标参数和上传的参数)
    dart_msgs::msg::DartLauncherParams target_dart_param_;
    dart_msgs::msg::DartLauncherParams target_dart_protocols_;
    dart_msgs::msg::DartLauncherStatus dart_status_;
    rclcpp::Time last_status_time_;
    bool mcu_online_;

    // 自适应弹速控制
    AdaptiveVelocityControl avc_;
    std::mutex avc_mutex_; // 用于自适应弹速控制的互斥锁

    bool block_param_update_ingame_;

    std::thread daemon_thread_;
    bool daemon_running_;
};

#endif