#ifndef NODE_LAUNCHER_DETECTOR_HPP
#define NODE_LAUNCHER_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <thread>
#include <string>
#include <unordered_map>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "camera_hal/camera_driver.hpp"
#include <detector/qrcode_detect.h>
#include <detector/greenlight_detect.h>
#include <std_msgs/msg/string.hpp>
#include <dart_msgs/msg/green_light.hpp>
#include "camera_hal/camera_dh.hpp"
#include "camera_hal/camera_lccv.hpp"

using namespace std::chrono_literals;
using namespace CameraHAL;

class NodeDartLauncherDetector : public rclcpp_lifecycle::LifecycleNode
{
private:
    std::shared_ptr<CameraDriver> camera_lccv_;
    std::shared_ptr<CameraDriver> camera_dh_;
    QRCodeDetectorWechat qr_detector_;
    std::shared_ptr<TopArmorDetect> greenlight_detector_;

    rclcpp_lifecycle::LifecyclePublisher<dart_msgs::msg::GreenLight>::SharedPtr greenlight_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr greenlight_image_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr qr_detect_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr qr_image_publisher_;

    std::shared_ptr<std::thread> lccv_thread_;
    std::shared_ptr<std::thread> dh_thread_;

    volatile bool running_;
    volatile bool lccv_enabled_;
    volatile bool dh_enabled_;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr callback_set_parameter_handle;

    void camera_thread(std::shared_ptr<CameraDriver> camera, const std::string &camera_name, bool is_qr_detection);
    cv::Mat perform_greenlight_detection(cv::Mat &frame, bool &is_detected, double &x, double &y);
    void on_parameter_event(const rclcpp::Parameter &param);
    bool load_and_open_camera(const std::string &camera_prefix, std::shared_ptr<CameraDriver> &camera_driver);

public:
    explicit NodeDartLauncherDetector(rclcpp::NodeOptions options);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &pre_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &pre_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &pre_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &pre_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &pre_state) override;
};

#endif // NODE_LAUNCHER_DETECTOR_HPP