#ifndef NODE_DART_LOGGER_HPP
#define NODE_DART_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <dart_msgs/msg/dart_launcher_status.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class NodeDartLogger : public rclcpp_lifecycle::LifecycleNode
{
public:
  NodeDartLogger(const rclcpp::NodeOptions & options);
  virtual ~NodeDartLogger();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

protected:
  // 生命周期节点回调函数
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // 订阅回调函数
  void status_callback(const dart_msgs::msg::DartLauncherStatus::SharedPtr msg);
  void log_callback(const std_msgs::msg::String::SharedPtr msg);
  
  // 检查状态消息中需要监控的字段是否有变化
  bool check_status_changes(const dart_msgs::msg::DartLauncherStatus * last_msg, 
                           const dart_msgs::msg::DartLauncherStatus::SharedPtr msg);

  // 订阅器
  rclcpp::Subscription<dart_msgs::msg::DartLauncherStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr log_sub_;

  // 保存上一次的状态消息，用于比较变化
  std::unique_ptr<dart_msgs::msg::DartLauncherStatus> last_status_msg_;
};

#endif // NODE_DART_LOGGER_HPP