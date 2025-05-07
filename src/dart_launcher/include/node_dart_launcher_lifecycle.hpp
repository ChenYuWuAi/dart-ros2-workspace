#ifndef NODE_DART_LOGGER_DOG_HPP
#define NODE_DART_LOGGER_DOG_HPP
#include <rclcpp/rclcpp.hpp>
#include <dart_msgs/msg/dart_launcher_status.hpp>
#include <dart_msgs/msg/judge.hpp>
#include <dart_msgs/msg/dart_param.hpp>

class NodeLoggerDog : public rclcpp::Node
{
private:
    rclcpp::Subscription<dart_msgs::msg::DartLauncherStatus>::SharedPtr dart_launcher_status_sub_;
    rclcpp::Subscription<dart_msgs::msg::Judge>::SharedPtr judge_sub_;
    rclcpp::Subscription<dart_msgs::msg::DartParam>::SharedPtr dart_param_present_sub_;
    rclcpp::Subscription<dart_msgs::msg::DartParam>::SharedPtr dart_param_sub_;
    
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;

public:
    NodeLoggerDog();

private:
    void dart_launcher_status_callback(const dart_msgs::msg::DartLauncherStatus::SharedPtr msg);
    void judge_callback(const dart_msgs::msg::Judge::SharedPtr msg);
    void dart_param_callback(const dart_msgs::msg::DartParam::SharedPtr msg);
    void on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

    bool check_changes(const dart_msgs::msg::Judge *last_msg, const dart_msgs::msg::Judge::SharedPtr msg);
    bool check_changes(const dart_msgs::msg::DartLauncherStatus *last_msg, const dart_msgs::msg::DartLauncherStatus::SharedPtr msg);
    bool check_changes(const dart_msgs::msg::DartParam *last_msg, const dart_msgs::msg::DartParam::SharedPtr msg);

    bool is_node_running(const std::string &node_name);
    void check_nodes();
    void restart_service();

    std::string exec(const char *cmd);

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> nodes_to_watch_;
    std::string service_name_;
    bool enable;
};
#endif // NODE_DART_LOGGER_DOG_HPP