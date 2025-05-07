#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include "dart_msgs/msg/green_light.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "detector/greenlight_detect.h"
#include <chrono>

class DetectPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
    TopArmorDetect detector_;
    rclcpp_lifecycle::LifecyclePublisher<dart_msgs::msg::GreenLight>::SharedPtr publisher_;
    // image_transport::Publisher image_publisher_;
    // image_transport::Subscriber image_subscriber_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

public:
    DetectPublisher(std::chrono::milliseconds interval, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(("dart_detector_node"),
                                          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &pre_state)
    {

        publisher_ = this->create_publisher<dart_msgs::msg::GreenLight>(
            "dart_detector/locate",
            rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable());
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("dart_detector/image", 1);
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image", 1,
                                                                               std::bind(&DetectPublisher::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "Node_detector on_configure is called for initial, pre state is %s",
                    pre_state.label().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &pre_state)
    {
        LifecycleNode::on_activate(pre_state);
        RCLCPP_INFO(this->get_logger(),
                    "Camera_Node on_activate is called, pre state is %s", pre_state.label().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &pre_state)
    {
        LifecycleNode::on_deactivate(pre_state);

        RCLCPP_INFO(this->get_logger(),
                    "Camera_Noder on_deactivate is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &pre_state)
    {
        // 释放资源，清空智能指针
        publisher_.reset();
        image_publisher_.reset();
        image_subscriber_.reset();
        RCLCPP_INFO(this->get_logger(),
                    "LifecycleTalker on_cleanup is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &pre_state)
    {
        // 释放资源，清空智能指针
        publisher_.reset();
        image_publisher_.reset();
        image_subscriber_.reset();
        RCLCPP_INFO(this->get_logger(),
                    "LifecycleTalker on_shutdown is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    cv::Mat perform_detection(cv::Mat &frame, bool &is_detected, double &x, double &y)
    {
        if (detector_.detect(frame))
        {
            is_detected = true;
            cv::Point2f center;
            detector_.getResult(center);
            x = center.x;
            y = center.y;
            ;
        }
        return detector_.drawRaw();
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &frame_msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(frame_msg, "bgr8")->image;
        RCLCPP_DEBUG(this->get_logger(), "Received image from camera");
        if (!frame.empty())
        {
            bool is_detected = false;
            double x = 0, y = 0;
            cv::Mat resultImg = perform_detection(frame, is_detected, x, y);
            auto message = dart_msgs::msg::GreenLight();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "dart_detector";
            message.is_detected = is_detected;
            message.location.x = x;
            message.location.y = y;
            message.location.z = 0.0;

            RCLCPP_DEBUG(this->get_logger(), "Publishing: is_detected='%s', x='%f', y='%f'", is_detected ? "true" : "false", x, y);
            publisher_->publish(message);

            std_msgs::msg::Header header;
            header.stamp = this->now();
            auto image_msg = cv_bridge::CvImage(header, "bgr8", resultImg).toImageMsg();
            image_publisher_->publish(*image_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get image from camera");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::chrono::milliseconds interval(1000 / 60);

    auto node = std::make_shared<DetectPublisher>(interval);

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
