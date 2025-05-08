#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <dart_msgs/msg/dart_launcher_status.hpp>
#include <dart_msgs/msg/dart_launcher_params.hpp>
#include <dart_comm_share/include/dart_launcher_default_value.hpp>
#include <std_msgs/msg/int32.hpp>

using dart_msgs::msg::DartLauncherStatus;

class MockDartMCU : public rclcpp::Node
{
public:
    MockDartMCU()
        : Node("dart_mcu")
    {
        // declare parameters for status fields
        this->declare_parameter("motor_yaw_online", false);
        this->declare_parameter("motor_trigger_online", false);
        this->declare_parameter("rc_online", false);
        this->declare_parameter("dart_state", 0);
        this->declare_parameter("dart_launch_process", 0);
        this->declare_parameter("motor_yaw_angle", 0);
        this->declare_parameter("motor_trigger_angle", 0);
        this->declare_parameter("last_launch_speed", 0.0);
        this->declare_parameter("last_launch_time", 0);

        // Set QoS to reliable
        rclcpp::QoS qos_reliable = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable();

        // Best effort QoS for publisher
        publisher_ = this->create_publisher<DartLauncherStatus>(
            "/dart_launcher_mcu/status", rclcpp::QoS(10).best_effort());
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MockDartMCU::on_timer, this));

        // initialize latest params and protocols with defaults
        defaultDartParams(current_params_);
        defaultDartProtocols(current_protocols_);
        // subscribers for params and protocols
        sub_params_ = this->create_subscription<dart_msgs::msg::DartLauncherParams>(
            "/dart_launcher_mcu/cmd_params", qos_reliable,
            [this](const dart_msgs::msg::DartLauncherParams::SharedPtr msg)
            {
                this->current_params_ = *msg;
                RCLCPP_INFO(this->get_logger(), "Received new params and updated.");
                RCLCPP_DEBUG(this->get_logger(), "Msg Timestamp: %d", msg->last_param_update_time);
            });
        sub_protocols_ = this->create_subscription<dart_msgs::msg::DartLauncherParams>(
            "/dart_launcher_mcu/cmd_protocols", qos_reliable,
            [this](const dart_msgs::msg::DartLauncherParams::SharedPtr msg)
            {
                this->current_protocols_ = *msg;
                RCLCPP_INFO(this->get_logger(), "Received new protocols and updated.");
                RCLCPP_DEBUG(this->get_logger(), "Msg Timestamp: %d", msg->last_param_update_time);
            });

        // Subscriber for buzzer sound effect commands
        sub_buzzer_ = this->create_subscription<std_msgs::msg::Int32>(
            "/dart_launcher_mcu/cmd_sound_effect", qos_reliable,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(), "Received buzzer sound effect command: %d", msg->data);
            });

        RCLCPP_INFO(this->get_logger(), "MockDartMCU node initialized.");
    }

private:
    void on_timer()
    {
        DartLauncherStatus msg;
        msg.header.stamp = this->now();
        msg.motor_yaw_online = this->get_parameter("motor_yaw_online").as_bool();
        msg.motor_trigger_online = this->get_parameter("motor_trigger_online").as_bool();
        msg.rc_online = this->get_parameter("rc_online").as_bool();
        msg.dart_state = this->get_parameter("dart_state").as_int();
        msg.dart_launch_process = this->get_parameter("dart_launch_process").as_int();
        msg.motor_yaw_angle = this->get_parameter("motor_yaw_angle").as_int();
        msg.motor_trigger_angle = this->get_parameter("motor_trigger_angle").as_int();
        msg.last_launch_speed = this->get_parameter("last_launch_speed").as_double();
        msg.last_launch_time = this->get_parameter("last_launch_time").as_int();
        // attach latest params and protocols
        msg.params = current_params_;
        msg.protocols = current_protocols_;
        publisher_->publish(msg);

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 
            1000, "Publishing DartLauncherStatus: %d", msg.header.stamp.sec);
    }

    rclcpp::Publisher<DartLauncherStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    dart_msgs::msg::DartLauncherParams current_params_;
    dart_msgs::msg::DartLauncherParams current_protocols_;
    rclcpp::Subscription<dart_msgs::msg::DartLauncherParams>::SharedPtr sub_params_;
    rclcpp::Subscription<dart_msgs::msg::DartLauncherParams>::SharedPtr sub_protocols_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_buzzer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockDartMCU>());
    rclcpp::shutdown();
    return 0;
}
