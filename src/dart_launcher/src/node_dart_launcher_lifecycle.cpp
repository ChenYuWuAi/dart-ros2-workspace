/**
 * @file node_dart_logger_dog.cpp
 * @brief 日志节点，以及看门狗
 */
#include <rclcpp/rclcpp.hpp>
#include <dart_msgs/msg/dart_launcher_status.hpp>
#include <dart_msgs/msg/judge.hpp>
#include <dart_msgs/msg/dart_param.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <dart_config.hpp>
#include <node_dart_logger_dog.hpp>

void NodeLoggerDog::on_parameter_change(const std::vector<rclcpp::Parameter> &parameters)
{
    for (const auto &parameter : parameters)
    {
        RCLCPP_INFO(this->get_logger(), "Parameter '%s' was changed to %s", parameter.get_name().c_str(), parameter.value_to_string().c_str());
    }
}

bool NodeLoggerDog::check_changes(const dart_msgs::msg::DartLauncherStatus *last_msg, const dart_msgs::msg::DartLauncherStatus::SharedPtr msg)
{
    static bool first_update = true;
    if (first_update)
    {
        first_update = false;
        return true;
    }
    if (last_msg->dart_launcher_online != msg->dart_launcher_online)
    {
        return true;
    }
    if (last_msg->judge_online != msg->judge_online)
    {
        return true;
    }
    if (last_msg->motor_dm_online != msg->motor_dm_online)
    {
        return true;
    }
    if (last_msg->motor_y_online != msg->motor_y_online)
    {
        return true;
    }
    if (last_msg->motor_ls_online != msg->motor_ls_online)
    {
        return true;
    }
    if (last_msg->dart_launch_process != msg->dart_launch_process)
    {
        return true;
    }
    if (last_msg->dart_state != msg->dart_state)
    {
        return true;
    }
    if (last_msg->rc_online != msg->rc_online)
    {
        return true;
    }
    for (size_t i = 0; i < 4; i++)
    {
        if (last_msg->motor_fw_online[i] != msg->motor_fw_online[i])
        {
            return true;
        }
    }
    return false;
}

bool NodeLoggerDog::check_changes(const dart_msgs::msg::Judge *last_msg, const dart_msgs::msg::Judge::SharedPtr msg)
{
    static bool first_update = true;
    if (first_update)
    {
        first_update = false;
        return true;
    }
    if (last_msg->dart_launch_opening_status != msg->dart_launch_opening_status)
    {
        return true;
    }
    if (last_msg->dart_remaining_time != msg->dart_remaining_time)
    {
        return true;
    }
    if (last_msg->game_progress != msg->game_progress)
    {
        return true;
    }
    if (last_msg->latest_launch_cmd_time != msg->latest_launch_cmd_time)
    {
        return true;
    }
    if (last_msg->stage_remain_time != msg->stage_remain_time)
    {
        return true;
    }
    return false;
}

bool NodeLoggerDog::check_changes(const dart_msgs::msg::DartParam *last_msg, const dart_msgs::msg::DartParam::SharedPtr msg)
{
    static bool first_update = true;
    if (first_update)
    {
        first_update = false;
        return true;
    }
    if (last_msg->target_yaw_angle != msg->target_yaw_angle)
    {
        return true;
    }
    if (last_msg->target_yaw_angle_offset != msg->target_yaw_angle_offset)
    {
        return true;
    }
    if (last_msg->target_fw_velocity != msg->target_fw_velocity)
    {
        return true;
    }
    if (last_msg->target_fw_velocity_offset != msg->target_fw_velocity_offset)
    {
        return true;
    }
    if (last_msg->target_fw_velocity_ratio != msg->target_fw_velocity_ratio)
    {
        return true;
    }
    if (last_msg->target_yaw_x_axis != msg->target_yaw_x_axis)
    {
        return true;
    }
    if (last_msg->target_distance != msg->target_distance)
    {
        return true;
    }
    if (last_msg->target_delta_height != msg->target_delta_height)
    {
        return true;
    }
    if (last_msg->auto_yaw_calibration != msg->auto_yaw_calibration)
    {
        return true;
    }
    if (last_msg->auto_fw_calibration != msg->auto_fw_calibration)
    {
        return true;
    }
    for (size_t i = 0; i < 4; i++)
    {
        if (last_msg->dart_selection[i] != msg->dart_selection[i])
        {
            return true;
        }
        if (last_msg->target_fw_velocity_launch_offset[i] != msg->target_fw_velocity_launch_offset[i])
        {
            return true;
        }
        if (last_msg->target_yaw_launch_angle_offset[i] != msg->target_yaw_launch_angle_offset[i])
        {
            return true;
        }
    }
    return false;
}

void NodeLoggerDog::dart_param_callback(const dart_msgs::msg::DartParam::SharedPtr msg)
{
    // 记录参数的改变
    static dart_msgs::msg::DartParam last_msg = *msg;
    // 如果有变动才记录
    bool is_changed = check_changes(&last_msg, msg);
    if (is_changed)
    {
        // 记录到日志
        RCLCPP_INFO(this->get_logger(), "DartParam changed to: target_yaw_angle %d, target_yaw_angle_offset %d, target_fw_velocity %d, target_fw_velocity_offset %d, target_fw_velocity_ratio %f, target_yaw_x_axis %f, target_distance %f, target_delta_height %f, auto_yaw_calibration %d, auto_fw_calibration %d",
                    msg->target_yaw_angle,
                    msg->target_yaw_angle_offset,
                    msg->target_fw_velocity,
                    msg->target_fw_velocity_offset,
                    msg->target_fw_velocity_ratio,
                    msg->target_yaw_x_axis,
                    msg->target_distance,
                    msg->target_delta_height,
                    msg->auto_yaw_calibration,
                    msg->auto_fw_calibration);

        RCLCPP_INFO(this->get_logger(), "Dart Selection %s %s %s %s", msg->dart_selection[0].c_str(), msg->dart_selection[1].c_str(), msg->dart_selection[2].c_str(), msg->dart_selection[3].c_str());
        RCLCPP_INFO(this->get_logger(), "Target FW Velocity offset(launch) %d %d %d %d", msg->target_fw_velocity_launch_offset[0], msg->target_fw_velocity_launch_offset[1], msg->target_fw_velocity_launch_offset[2], msg->target_fw_velocity_launch_offset[3]);
        RCLCPP_INFO(this->get_logger(), "Target YAW Angle offset(launch) %d %d %d %d", msg->target_yaw_launch_angle_offset[0], msg->target_yaw_launch_angle_offset[1], msg->target_yaw_launch_angle_offset[2], msg->target_yaw_launch_angle_offset[3]);
        last_msg = *msg;
    }
}

void NodeLoggerDog::dart_launcher_status_callback(const dart_msgs::msg::DartLauncherStatus::SharedPtr msg)
{
    // 记录异常掉线情况、电池电压、发射参数的改变
    static dart_msgs::msg::DartLauncherStatus last_msg = *msg;
    // 如果有变动才记录
    bool is_changed = check_changes(&last_msg, msg);
    if (is_changed)
    {
        // 记录到日志
        RCLCPP_INFO(this->get_logger(), "DartLauncher %s, Judge %s, Motor DM %s, Motor Y %s, Motor LS %s, Motor FW %s %s %s %s, RC %s.",
                    msg->dart_launcher_online ? "Online" : "Offline",
                    msg->judge_online ? "Online" : "Offline",
                    msg->motor_dm_online ? "Online" : "Offline",
                    msg->motor_y_online ? "Online" : "Offline",
                    msg->motor_ls_online ? "Online" : "Offline",
                    msg->motor_fw_online[0] ? "Online" : "Offline",
                    msg->motor_fw_online[1] ? "Online" : "Offline",
                    msg->motor_fw_online[2] ? "Online" : "Offline",
                    msg->motor_fw_online[3] ? "Online" : "Offline",
                    msg->rc_online ? "Online" : "Offline");
        RCLCPP_INFO(this->get_logger(), "Bus Voltage: %f %f %f %f", msg->bus_voltage[0], msg->bus_voltage[1], msg->bus_voltage[2], msg->bus_voltage[3]);
        RCLCPP_INFO(this->get_logger(), "DartLaunchProcess %d, DartState %d.",
                    msg->dart_launch_process,
                    msg->dart_state);
        last_msg = *msg;
    }
}

void NodeLoggerDog::judge_callback(const dart_msgs::msg::Judge::SharedPtr msg)
{
    // 记录裁判系统的状态
    static dart_msgs::msg::Judge last_msg = *msg;
    // 如果有变动才记录
    bool is_changed = check_changes(&last_msg, msg);
    if (is_changed)
    {
        // 记录到日志
        RCLCPP_INFO(this->get_logger(), "DartLaunchOpeningStatus %d, DartRemainingTime %d, GameProgress %d, LatestLaunchCmdTime %d, StageRemainTime %d.",
                    msg->dart_launch_opening_status,
                    msg->dart_remaining_time,
                    msg->game_progress,
                    msg->latest_launch_cmd_time,
                    msg->stage_remain_time);
        last_msg = *msg;
    }
}

void NodeLoggerDog::check_nodes()
{
    static bool first_update = true;
    if (first_update)
    {
        timer_->cancel();
        first_update = false;
        timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&NodeLoggerDog::check_nodes, this));
    }
    // 更新参数
    nodes_to_watch_ = this->get_parameter("nodes_to_watch").as_string_array();
    service_name_ = this->get_parameter("service_name").as_string();
    enable = this->get_parameter("watchdog_on").as_bool();

    if (enable)
    {
        bool all_nodes_running = true;

        for (const auto &node_name : nodes_to_watch_)
        {
            if (!is_node_running(node_name))
            {
                RCLCPP_WARN(this->get_logger(), "Node %s is not running, trying to restart", node_name.c_str());
                all_nodes_running = false;
            }
        }

        if (!all_nodes_running)
        {
            RCLCPP_INFO(this->get_logger(), "Restarting systemd service...");
            restart_service();
        }
    }
}

bool NodeLoggerDog::is_node_running(const std::string &node_name)
{
    auto names = this->get_node_names();

    return !(std::find(names.begin(), names.end(), "/" + node_name) == names.end());
}

std::string NodeLoggerDog::exec(const char *cmd)
{
    char buffer[128];
    std::string result = "";
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        RCLCPP_ERROR(this->get_logger(), "popen() failed.");
        return "";
    }
    while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr)
    {
        result += buffer;
    }
    return result;
}

// 类定义
NodeLoggerDog::NodeLoggerDog() : Node("dart_logger_dog")
{
    DartConfig::declareParameters(*this);
    this->declare_parameter("watchdog_on", true);
    this->declare_parameter("service_name", "dart_ros2_run.service");
    /*/dart_config
    /can_agent
    /lvgl_ui
    /dart_detector_node
    /dart_detector_publisher
    /camera
    /dart_logger_dog*/
    this->declare_parameter("nodes_to_watch", std::vector<std::string>{"dart_config", "can_agent", "lvgl_ui", "dart_detector_node", "dart_detector_publisher", "camera"});

    param_cb_handle_ =
        this->add_post_set_parameters_callback(std::bind(&NodeLoggerDog::on_parameter_change, this, std::placeholders::_1));

    nodes_to_watch_ = this->get_parameter("nodes_to_watch").as_string_array();
    service_name_ = this->get_parameter("service_name").as_string();
    enable = this->get_parameter("watchdog_on").as_bool();

    // callback register
    dart_launcher_status_sub_ = this->create_subscription<dart_msgs::msg::DartLauncherStatus>(
        "/dart_launcher/dart_launcher_status",
        10, std::bind(&NodeLoggerDog::dart_launcher_status_callback, this, std::placeholders::_1));

    dart_param_present_sub_ = this->create_subscription<dart_msgs::msg::DartParam>(
        "/dart_launcher/dart_launcher_present_param",
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(), std::bind(&NodeLoggerDog::dart_param_callback, this, std::placeholders::_1));

    dart_param_sub_ = this->create_subscription<dart_msgs::msg::DartParam>(
        "/dart_launcher/dart_launcher_param_cmd",
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(), std::bind(&NodeLoggerDog::dart_param_callback, this, std::placeholders::_1));

    judge_sub_ = this->create_subscription<dart_msgs::msg::Judge>(
        "/dart_launcher/judge",
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(), std::bind(&NodeLoggerDog::judge_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&NodeLoggerDog::check_nodes, this));

    RCLCPP_INFO(this->get_logger(), "Logger & Watchdog is on.");
}

void NodeLoggerDog::restart_service()
{
    std::string command = "sudo systemctl restart " + service_name_;
    int result = system(command.c_str());

    if (result == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Successfully restarted %s.", service_name_.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to restart service %s.", service_name_.c_str());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeLoggerDog>());
    rclcpp::shutdown();
    return 0;
}
