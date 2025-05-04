/**
 * @file node_dart_param_gateway.hpp
 * @brief NodeDartParamGateway 类头文件
 */
#ifndef NODE_DART_PARAM_GATEWAY_HPP
#define NODE_DART_PARAM_GATEWAY_HPP
// ROS2 Lifecycle Node
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// Std_msg
#include <std_msgs/msg/string.hpp>

// Dart_msg
#include <dart_msgs/msg/dart_launcher_params.hpp>
#include <dart_msgs/msg/dart_launcher_status.hpp>
#include <dart_msgs/msg/green_light.hpp>

// ROS2 Service
#include <std_srvs/srv/empty.hpp>

// Json
#include <nlohmann/json.hpp>

using json = nlohmann::json;

#ifndef CONFIG_PATH
#define CONFIG_PATH "/home/chenyu/dart-ros2-workspace/install/dart_launcher/share/dart_launcher/config" + "dart_launcher_params.json"
#endif

class NodeDartParamGateway : public rclcpp_lifecycle::LifecycleNode
{
public:
    NodeDartParamGateway();
    ~NodeDartParamGateway();
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

private:
    // ROS2 Lifecycle Node
    rclcpp::Node::SharedPtr node_;

    // Publisher
    rclcpp::Publisher<dart_msgs::msg::DartLauncherParams>::SharedPtr dart_param_pub_;

    // Subscriber
    // for string json params
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dart_qr_param_sub_;
    // for launcher param subscription
    rclcpp::Subscription<dart_msgs::msg::DartLauncherParams>::SharedPtr dart_param_sub_;
    // for greenlight detector
    rclcpp::Subscription<dart_msgs::msg::GreenLight>::SharedPtr greenlight_sub_;

    // Dart_param
    dart_msgs::msg::DartLauncherParams current_dart_param_, target_dart_param_;
    // Dart_status
    dart_msgs::msg::DartLauncherStatus dart_status_;
};

#endif