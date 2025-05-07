#ifndef NODE_LVGL_UI_HPP
#define NODE_LVGL_UI_HPP

#include <rclcpp/rclcpp.hpp>

#include "lvgl/lvgl.h"
#include "lv_drivers/display/fbdev.h"
#include "lv_drivers/indev/evdev.h"
#include <dart_msgs/msg/dart_launcher_status.hpp>
#include <dart_msgs/msg/dart_param.hpp>
#include <dart_msgs/msg/green_light.hpp>
#include <dart_msgs/msg/judge.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <dart_algorithm.hpp>

#include "gui_guider.h"
#include "events_init.h"
#include "dart_config.hpp"

#include <unistd.h>
#include <thread>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <string>
#include "get_ip.hpp"
#include <filesystem>

#ifndef YAML_PATH
#define YAML_PATH "/home/chenyu/dart24_ws/install/dart_launcher/share/dart_launcher/config" + "dart_config.yaml"
#endif

namespace fs = std::filesystem;

#define DISP_BUF_SIZE (600 * 1024)

// ROS 2 Interface
class NodeLVGLUI : public rclcpp::Node
{
public:
    bool param_set_callback_disabled = false;
    NodeLVGLUI();
    void loadParametersfromGUI(bool update_to_ros_param = true);
    void calibration_yaw();
    void calibration_fw(bool set_parameter = true);
    std::filesystem::file_time_type last_write_time;
    std::shared_ptr<DartAlgorithm::DartDataBase> dart_db_;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr callback_set_parameter_handle;
    std::vector<int>
        target_yaw_launch_angle_offset = {0, 0, 0, 0};

private:
    std::mutex mutex_ui_;
    rclcpp::TimerBase::SharedPtr timer_[3];
    rclcpp::Publisher<dart_msgs::msg::DartParam>::SharedPtr dart_launcher_cmd_pub_;
    rclcpp::Subscription<dart_msgs::msg::DartLauncherStatus>::SharedPtr dart_launcher_status_sub_;
    rclcpp::Subscription<dart_msgs::msg::DartParam>::SharedPtr dart_launcher_present_param_sub_;
    rclcpp::Subscription<dart_msgs::msg::GreenLight>::SharedPtr green_light_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cv_image_sub_;
    rclcpp::Subscription<dart_msgs::msg::Judge>::SharedPtr judge_sub_;

    dart_msgs::msg::Judge judge_msg_;

    std::shared_ptr<dart_msgs::msg::DartLauncherStatus> dart_launcher_status_;
    std::shared_ptr<dart_msgs::msg::GreenLight> green_light_;

    std::vector<lv_obj_t *> Main_list_darts_items_;

    void update_dart_launcher_status_callback(dart_msgs::msg::DartLauncherStatus::SharedPtr msg);
    void update_dart_launcher_present_param_callback(dart_msgs::msg::DartParam::SharedPtr msg);
    void update_green_light_callback(dart_msgs::msg::GreenLight::SharedPtr msg);
    void update_dart_database();
    void update_ip_address();
    void update_parameters_to_gui();
    void callback_set_parameter(const std::vector<rclcpp::Parameter> &parameters);
    void update_cv_image(sensor_msgs::msg::Image::SharedPtr msg);
    lv_color_t *img_buf;
    void timer_callback_ui();
    template <typename Func>
    friend void with_ui_lock(std::shared_ptr<NodeLVGLUI> node, Func func);
};

template <typename Func>
void with_ui_lock(std::shared_ptr<NodeLVGLUI> node, Func func)
{
    std::lock_guard<std::mutex> lock(node->mutex_ui_);
    func(); // 调用传递的lambda表达式
}

extern std::shared_ptr<NodeLVGLUI> node;

#endif