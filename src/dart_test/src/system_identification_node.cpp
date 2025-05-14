/**
 * @file system_identification_node.cpp
 * @brief 系统辨识节点，生成阶跃输入并记录系统响应
 */

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "dart_msgs/msg/dart_launcher_params.hpp"
#include "dart_msgs/msg/dart_launcher_status.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * 系统辨识模式
 */
enum class IdentificationMode
{
    STEP_RESPONSE,  // 阶跃响应模式
    FREQUENCY_SWEEP // 频率扫描模式
};

/**
 * @class SystemIdentificationNode
 * @brief ROS2节点类，用于系统辨识
 *
 * 该节点生成阶跃输入或频率扫描输入到/dart_launcher_mcu/cmd_params.target_yaw_angle，并记录
 * /dart_launcher_mcu/status.current_yaw_angle的响应，然后将数据保存为CSV文件
 */
class SystemIdentificationNode : public rclcpp::Node
{
public:
    SystemIdentificationNode()
        : Node("system_identification_node")
    {
        // 初始化参数
        this->declare_parameter("identification_mode", "step");                    // 辨识模式: "step" 或 "sweep"
        this->declare_parameter("step_size", 10000);                               // 阶跃大小
        this->declare_parameter("step_interval", 5.0);                             // 阶跃间隔时间(秒)
        this->declare_parameter("min_angle", 30000);                               // 最小角度值 (确保在10000~110000范围内)
        this->declare_parameter("max_angle", 90000);                               // 最大角度值 (确保在10000~110000范围内)
        this->declare_parameter("csv_filename", "system_identification_data.csv"); // 输出文件名

        // 频率扫描相关参数
        this->declare_parameter("sweep_amplitude", 10000); // 扫描幅值
        this->declare_parameter("sweep_center", 60000);    // 扫描中心值
        this->declare_parameter("sweep_min_freq", 0.01);   // 最小扫描频率(Hz)
        this->declare_parameter("sweep_max_freq", 2.0);    // 最大扫描频率(Hz)
        this->declare_parameter("sweep_bandwidth", 0.0);   // 带宽(Hz)，0表示使用min_freq到max_freq的全带宽
        this->declare_parameter("sweep_duration", 60.0);   // 扫描持续时间(秒)
        this->declare_parameter("sample_rate", 100.0);     // 采样率(Hz)

        // 获取参数
        auto mode_str = this->get_parameter("identification_mode").as_string();
        mode_ = (mode_str == "sweep") ? IdentificationMode::FREQUENCY_SWEEP : IdentificationMode::STEP_RESPONSE;

        step_size_ = this->get_parameter("step_size").as_int();
        step_interval_ = this->get_parameter("step_interval").as_double();
        min_angle_ = this->get_parameter("min_angle").as_int();
        max_angle_ = this->get_parameter("max_angle").as_int();
        csv_filename_ = this->get_parameter("csv_filename").as_string();

        sweep_amplitude_ = this->get_parameter("sweep_amplitude").as_int();
        sweep_center_ = this->get_parameter("sweep_center").as_int();
        sweep_min_freq_ = this->get_parameter("sweep_min_freq").as_double();
        sweep_max_freq_ = this->get_parameter("sweep_max_freq").as_double();
        sweep_bandwidth_ = this->get_parameter("sweep_bandwidth").as_double();
        sweep_duration_ = this->get_parameter("sweep_duration").as_double();
        sample_rate_ = this->get_parameter("sample_rate").as_double();

        // 确保角度范围在有效区间
        if (min_angle_ < 10000)
            min_angle_ = 10000;
        if (max_angle_ > 110000)
            max_angle_ = 110000;

        // 确保扫描中心和幅值在有效范围内
        if (sweep_center_ - sweep_amplitude_ < 10000)
            sweep_amplitude_ = sweep_center_ - 10000;
        if (sweep_center_ + sweep_amplitude_ > 110000)
            sweep_amplitude_ = 110000 - sweep_center_;

        // 创建发布者和订阅者
        pub_cmd_params_ = this->create_publisher<dart_msgs::msg::DartLauncherParams>(
            "/dart_launcher_mcu/cmd_params", 10);

        sub_status_ = this->create_subscription<dart_msgs::msg::DartLauncherStatus>(
            "/dart_launcher_mcu/status",
            rclcpp::QoS(100).best_effort(),
            std::bind(&SystemIdentificationNode::status_callback, this, _1));

        // 打开CSV文件用于数据记录
        data_file_.open(csv_filename_);
        if (!data_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开CSV文件：%s", csv_filename_.c_str());
            return;
        }

        // 写入CSV文件头
        data_file_ << "timestamp,target_yaw_angle,current_yaw_angle";
        if (mode_ == IdentificationMode::FREQUENCY_SWEEP)
        {
            data_file_ << ",frequency";
        }
        data_file_ << std::endl;

        // 初始化变量
        if (mode_ == IdentificationMode::STEP_RESPONSE)
        {
            // 阶跃测试模式
            current_target_ = min_angle_;
            is_step_up_ = true;

            // 发送初始目标角度
            send_target_angle(current_target_);

            // 启动阶跃定时器
            step_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(step_interval_),
                std::bind(&SystemIdentificationNode::step_timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "系统辨识节点已启动 - 阶跃响应模式");
            RCLCPP_INFO(this->get_logger(), "角度范围: %d - %d, 阶跃大小: %d, 间隔: %.1f秒",
                        min_angle_, max_angle_, step_size_, step_interval_);
        }
        else
        {
            // 频率扫描模式
            sweep_start_time_ = this->now();

            // 设置较高频率的定时器用于生成频率扫描信号
            sweep_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / sample_rate_),
                std::bind(&SystemIdentificationNode::sweep_timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "系统辨识节点已启动 - 频率扫描模式");
            if (sweep_bandwidth_ > 0)
            {
                double center_freq_log = sqrt(sweep_min_freq_ * sweep_max_freq_);
                RCLCPP_INFO(this->get_logger(), "中心角度: %d, 幅值: %d, 频率中心: %.2f Hz, 带宽: %.2f Hz, 持续时间: %.1f秒",
                            sweep_center_, sweep_amplitude_, center_freq_log, sweep_bandwidth_, sweep_duration_);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "中心角度: %d, 幅值: %d, 频率范围: %.2f-%.2f Hz, 持续时间: %.1f秒",
                            sweep_center_, sweep_amplitude_, sweep_min_freq_, sweep_max_freq_, sweep_duration_);
            }
        }
    }

    ~SystemIdentificationNode()
    {
        if (data_file_.is_open())
        {
            data_file_.close();
            RCLCPP_INFO(this->get_logger(), "数据已保存到文件: %s", csv_filename_.c_str());
        }
    }

private:
    /**
     * @brief 处理状态回调函数
     * @param msg 状态消息
     */
    void status_callback(const dart_msgs::msg::DartLauncherStatus::SharedPtr msg)
    {
        auto now = this->now();
        auto current_target = msg->params.primary_yaw;
        current_yaw_angle_ = msg->motor_yaw_angle;

        // 获取msg timestamp
        auto msg_timestamp = msg->header.stamp;
        uint64_t timestamp = msg_timestamp.sec * 1000000ULL + msg_timestamp.nanosec / 1000ULL; // 转换为微秒

        // 记录数据到CSV文件
        data_file_ << timestamp << ","
                   << current_target << ","
                   << current_yaw_angle_;

        // 如果是频率扫描模式，记录当前频率
        if (mode_ == IdentificationMode::FREQUENCY_SWEEP)
        {
            double elapsed = (now - sweep_start_time_).seconds();
            double current_freq = calculate_frequency(elapsed);
            data_file_ << "," << current_freq;
        }
        data_file_ << std::endl;
    }

    /**
     * @brief 随机生成新的阶跃角度
     */
    void randomly_generate_new_step_angle()
    {
        // 随机生成新的阶跃角度
        current_target_ = min_angle_ + rand() % (max_angle_ - min_angle_);
        RCLCPP_INFO(this->get_logger(), "随机生成新的阶跃角度: %d", current_target_);
    }

    /**
     * @brief 阶跃定时器回调函数，生成阶跃输入
     */
    void step_timer_callback()
    {
        // 生成阶跃信号
        if (is_step_up_)
        {
            randomly_generate_new_step_angle();
        }
        else
        {
            current_target_ = min_angle_;
        }

        // 翻转阶跃方向
        is_step_up_ = !is_step_up_;

        // 发送目标角度
        send_target_angle(current_target_);

        RCLCPP_INFO(this->get_logger(), "阶跃变化: 目标角度设置为 %d", current_target_);
    }

    /**
     * @brief 频率扫描定时器回调函数，生成频率扫描输入
     */
    void sweep_timer_callback()
    {
        auto now = this->now();
        double elapsed = (now - sweep_start_time_).seconds();

        // 检查是否达到扫描持续时间
        if (elapsed > sweep_duration_)
        {
            // 停止扫描，退出节点
            RCLCPP_INFO(this->get_logger(), "频率扫描完成，持续时间 %.1f 秒", elapsed);
            rclcpp::shutdown();
            return;
        }

        // 计算当前频率 - 对数扫描
        double current_freq = calculate_frequency(elapsed);

        // 计算当前角度 - 正弦波
        double angle = sweep_center_ + sweep_amplitude_ * sin(2.0 * M_PI * current_freq * elapsed);
        current_target_ = static_cast<int32_t>(angle);

        // 发送目标角度
        send_target_angle(current_target_);

        // 每秒打印一次当前频率信息
        static int log_count = 0;
        log_count++;
        if (log_count >= sample_rate_)
        {
            RCLCPP_INFO(this->get_logger(), "当前扫描频率: %.3f Hz, 目标角度: %d", current_freq, current_target_);
            log_count = 0;
        }
    }

    /**
     * @brief 计算当前扫描频率（对数扫描）
     * @param elapsed 已经过的时间(秒)
     * @return 当前频率值(Hz)
     */
    double calculate_frequency(double elapsed)
    {
        // 确定实际的频率范围
        double actual_min_freq = sweep_min_freq_;
        double actual_max_freq = sweep_max_freq_;

        // 如果指定了带宽且大于0，则计算基于中心频率的频率范围
        if (sweep_bandwidth_ > 0)
        {
            // 计算对数中心频率
            double center_freq_log = sqrt(sweep_min_freq_ * sweep_max_freq_);
            // 基于带宽计算新的最小和最大频率
            double half_bandwidth = sweep_bandwidth_ / 2.0;
            actual_min_freq = std::max(sweep_min_freq_, center_freq_log - half_bandwidth);
            actual_max_freq = std::min(sweep_max_freq_, center_freq_log + half_bandwidth);
        }

        // 线性时间到对数频率的映射 (对数扫描)
        double sweep_progress = elapsed / sweep_duration_;
        // 确保进度在0-1范围内
        sweep_progress = std::min(1.0, std::max(0.0, sweep_progress));
        return actual_min_freq * pow(actual_max_freq / actual_min_freq, sweep_progress);
    }

    /**
     * @brief 发送目标角度
     * @param target_angle 目标角度值
     */
    void send_target_angle(int32_t target_angle)
    {
        auto cmd_params = std::make_unique<dart_msgs::msg::DartLauncherParams>();
        cmd_params->primary_yaw = target_angle;
        cmd_params->primary_force = 10000000;                                  // 设置为一个默认值
        // cmd_params->last_param_update_time = this->now().nanoseconds() / 1000000; // 转换为毫秒

        pub_cmd_params_->publish(std::move(cmd_params));
    }

    // 辨识模式
    IdentificationMode mode_;

    // 发布者和订阅者
    rclcpp::Publisher<dart_msgs::msg::DartLauncherParams>::SharedPtr pub_cmd_params_;
    rclcpp::Subscription<dart_msgs::msg::DartLauncherStatus>::SharedPtr sub_status_;

    // 定时器
    rclcpp::TimerBase::SharedPtr step_timer_;
    rclcpp::TimerBase::SharedPtr sweep_timer_;

    // 参数 - 通用
    int min_angle_;
    int max_angle_;
    std::string csv_filename_;

    // 参数 - 阶跃响应
    int step_size_;
    double step_interval_;

    // 参数 - 频率扫描
    int sweep_amplitude_;           // 扫描幅值
    int sweep_center_;              // 扫描中心值
    double sweep_min_freq_;         // 最小扫描频率(Hz)
    double sweep_max_freq_;         // 最大扫描频率(Hz)
    double sweep_bandwidth_;        // 带宽(Hz)
    double sweep_duration_;         // 扫描持续时间(秒)
    double sample_rate_;            // 采样率(Hz)
    rclcpp::Time sweep_start_time_; // 扫描开始时间

    // 状态变量
    int32_t current_target_;
    int32_t current_yaw_angle_;
    bool is_step_up_;

    // 数据记录
    std::ofstream data_file_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemIdentificationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}