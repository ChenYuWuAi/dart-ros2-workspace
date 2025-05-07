/**
 * @file dart_flysystem_hardware_actuator.cpp
 * @brief 飞镖飞行系统硬件控制库
 */

#include "dart_flysystem_hardware/dart_flysystem_hardware_actuator.hpp"
#include "rclcpp/rclcpp.hpp"

// PWM硬件控制库
#include "dart_flysystem_hardware/linux_pwm.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/time.h>
#include <linux/input.h>
#include "buzzer_songs.h"

#include <vector>

namespace dart_flysystem_hardware
{
    // PWM->Servo角度转换
#define PWM_SERVO_PWM_PERIOD_NS 20000000.0     // 20ms
#define PWM_SERVO_MIN_NS 500000.0              // 0.5ms
#define PWM_SERVO_MAX_NS 2500000.0             // 2.5ms
#define SERVO_MAX_ANGLE 3.14159265358979323846 // Pi

    static bool setAngleToPwm(const double angle, const std::shared_ptr<linuxPWM::LinuxPwm> &pwm)
    {
        // 转化弧度到PWM脉冲宽度
        return pwm->setDutyCycle(
            PWM_SERVO_MIN_NS + (PWM_SERVO_MAX_NS - PWM_SERVO_MIN_NS) / SERVO_MAX_ANGLE * angle);
    }

    hardware_interface::CallbackReturn
    DartFlySystemHardwareActuator::on_init(const hardware_interface::HardwareInfo &info)
    {
        // 初始化超类
        if (
            SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 设置offset角度
        if (info_.hardware_parameters.find("offset_angle") != info_.hardware_parameters.end())
        {
            offset_angle_ = std::stod(info_.hardware_parameters.at("offset_angle"));
        }
        else
        {
            offset_angle_ = 0;
        }

        RCLCPP_INFO(
            get_logger(),
            "Offset angle is %f", offset_angle_);

        // Debug开关读取
        if (const auto debug = info_.hardware_parameters.find("debug"); debug != info_.hardware_parameters.end())
        {
            RCLCPP_INFO(
                get_logger(),
                "Debug mode is %s", debug->second.c_str());
            debug_ = debug->second == "true";
        }
        else
        {
            debug_ = false;
        }

        // 遍历所有joint，按照接口类型区分servo和throttle
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // 期望每个joint只有一个command和一个state接口
            if (joint.command_interfaces.size() != 1 || joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(get_logger(),
                             "Joint '%s' have %zu command interfaces and %zu state interfaces found. 1 expected.",
                             joint.name.c_str(), joint.command_interfaces.size(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // 当接口为servo（position接口）
            if (joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
            {
                // 原有servo流程
                try
                {
                    const auto pwm_chip = std::stoi(joint.parameters.at("pwm_chip"));
                    const auto pwm_channel = std::stoi(joint.parameters.at("pwm_channel"));
                    pwm_map_[joint.name] = std::make_shared<linuxPWM::LinuxPwm>();
                    if (!pwm_map_[joint.name]->begin(pwm_chip, pwm_channel))
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to export PWM for joint '%s'", joint.name.c_str());
                        return hardware_interface::CallbackReturn::ERROR;
                    }
                    RCLCPP_INFO(get_logger(),
                                "Joint '%s' initialized, PWM chip: %d, channel: %d",
                                joint.name.c_str(), pwm_chip, pwm_channel);
                }
                catch (const std::out_of_range &e)
                {
                    RCLCPP_FATAL(get_logger(),
                                 "%s, Joint '%s' has invalid PWM chip or channel.", e.what(), joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                angle_map_[joint.name] = 0;
                min_angle_map_[joint.name] = joint.parameters.find("min") != joint.parameters.end()
                                                 ? std::stof(joint.parameters.at("min"))
                                                 : 0;
                max_angle_map_[joint.name] = joint.parameters.find("max") != joint.parameters.end()
                                                 ? std::stof(joint.parameters.at("max"))
                                                 : SERVO_MAX_ANGLE;
                RCLCPP_INFO(get_logger(),
                            "Joint '%s' initialized, min: %f, max: %f",
                            joint.name.c_str(), min_angle_map_[joint.name], max_angle_map_[joint.name]);
            }
            // 当接口为throttle（velocity接口）
            else if (joint.command_interfaces[0].name == "velocity")
            {
                try
                {
                    const auto pwm_chip = std::stoi(joint.parameters.at("pwm_chip"));
                    const auto pwm_channel = std::stoi(joint.parameters.at("pwm_channel"));
                    pwm_map_[joint.name] = std::make_shared<linuxPWM::LinuxPwm>();
                    if (!pwm_map_[joint.name]->begin(pwm_chip, pwm_channel))
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to export PWM for throttle joint '%s'", joint.name.c_str());
                        return hardware_interface::CallbackReturn::ERROR;
                    }
                    RCLCPP_INFO(get_logger(),
                                "Throttle joint '%s' initialized, PWM chip: %d, channel: %d",
                                joint.name.c_str(), pwm_chip, pwm_channel);
                }
                catch (const std::out_of_range &e)
                {
                    RCLCPP_FATAL(get_logger(),
                                 "%s, Throttle joint '%s' has invalid PWM chip or channel.",
                                 e.what(), joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // 加载默认velocity及PWM占空比范围（单位百分比）
                throttle_velocity_ =
                    (joint.parameters.find("default_velocity") != joint.parameters.end())
                        ? std::stod(joint.parameters.at("default_velocity"))
                        : 0.0;
                throttle_min_percentage_ =
                    (joint.parameters.find("min_velocity_duty_cycle_percentage") != joint.parameters.end())
                        ? std::stod(joint.parameters.at("min_velocity_duty_cycle_percentage"))
                        : 50.0;
                throttle_max_percentage_ =
                    (joint.parameters.find("max_velocity_duty_cycle_percentage") != joint.parameters.end())
                        ? std::stod(joint.parameters.at("max_velocity_duty_cycle_percentage"))
                        : 95.0;

                // 计算pwm period ns
                throttle_pwm_period_ns = (joint.parameters.find("pwm_frequency") != joint.parameters.end())
                                             ? 1000000000 / std::stoi(joint.parameters.at("pwm_frequency"))
                                             : 2000000;

                RCLCPP_INFO(get_logger(),
                            "Throttle joint '%s' initialized, default_velocity: %f, duty cycle range: %f%% ~ %f%%",
                            joint.name.c_str(), throttle_velocity_,
                            throttle_min_percentage_, throttle_max_percentage_);
            }
            else
            {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has unknown command interface '%s'.",
                             joint.name.c_str(), joint.command_interfaces[0].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface> DartFlySystemHardwareActuator::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (const auto &joint : info_.joints)
        {
            if (joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
                command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &angle_map_[joint.name]);
            else if (joint.command_interfaces[0].name == "velocity")
                command_interfaces.emplace_back(joint.name, "velocity", &throttle_velocity_);

            RCLCPP_INFO(get_logger(), "Joint '%s' command interface '%s' exported",
                        joint.name.c_str(), joint.command_interfaces[0].name.c_str());
        }
        return command_interfaces;
    }

    std::vector<hardware_interface::StateInterface> DartFlySystemHardwareActuator::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (const auto &joint : info_.joints)
        {
            if (joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION)
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &angle_map_[joint.name]);
            else if (joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)
                state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &throttle_velocity_);

            RCLCPP_INFO(get_logger(), "Joint '%s' state interface '%s' exported",
                        joint.name.c_str(), joint.state_interfaces[0].name.c_str());
        }
        return state_interfaces;
    }

    hardware_interface::CallbackReturn
    DartFlySystemHardwareActuator::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        for (auto &joint : info_.joints)
        {
            RCLCPP_INFO(get_logger(), "Configuring joint '%s'", joint.name.c_str());
            if (joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
            {
                if (!pwm_map_[joint.name]->setPeriod(PWM_SERVO_PWM_PERIOD_NS))
                {
                    RCLCPP_ERROR(get_logger(), "Failed to set period for joint '%s'", joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                angle_map_[joint.name] = (joint.parameters.find("initial_angle") != joint.parameters.end())
                                             ? std::stoi(joint.parameters.at("initial_angle"))
                                             : 0;
                angle_map_[joint.name] = std::max(min_angle_map_[joint.name],
                                                  std::min(angle_map_[joint.name], max_angle_map_[joint.name]));
                if (!setAngleToPwm(angle_map_[joint.name] + offset_angle_, pwm_map_[joint.name]))
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to set initial angle %f for joint '%s'",
                                 angle_map_[joint.name] + offset_angle_, joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
            else if (joint.command_interfaces[0].name == "velocity")
            {
                // 对throttle关节，设置周期，对应THROTTLE_PWM_PERIOD_NS
                if (!pwm_map_[joint.name]->setPeriod(throttle_pwm_period_ns))
                {
                    RCLCPP_ERROR(get_logger(), "Failed to set period for throttle joint '%s'", joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                // 复位输出50%占空比，即占空比 = min_percentage
                double min_percent = throttle_min_percentage_;
                unsigned int duty_ns = static_cast<unsigned int>(throttle_pwm_period_ns * (min_percent / 100.0));
                if (!pwm_map_[joint.name]->setDutyCycle(duty_ns))
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to set initial duty cycle for throttle joint '%s'",
                                 joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DartFlySystemHardwareActuator::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        for (auto &joint : info_.joints)
        {
            if (joint.command_interfaces[0].name == "velocity")
            {
                // 复位输出50%占空比，即占空比 = min_percentage
                double min_percent = throttle_min_percentage_;
                unsigned int duty_ns = static_cast<unsigned int>(throttle_pwm_period_ns * (min_percent / 100.0));
                if (!pwm_map_[joint.name]->setDutyCycle(duty_ns))
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to set initial duty cycle for throttle joint '%s'",
                                 joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
            // 启用PWM
            if (!pwm_map_[joint.name]->enable())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to enable PWM for joint '%s'",
                    joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        // 等待时间供电调复位
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DartFlySystemHardwareActuator::read(const rclcpp::Time &time,
                                                                        const rclcpp::Duration &period)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DartFlySystemHardwareActuator::write(const rclcpp::Time &time,
                                                                         const rclcpp::Duration &period)
    {
        for (auto &joint : info_.joints)
        {
            if (joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
            {
                // Servo关节处理（保持原有逻辑）
                angle_map_[joint.name] = std::max(min_angle_map_[joint.name],
                                                  std::min(angle_map_[joint.name], max_angle_map_[joint.name]));
                double angle = angle_map_[joint.name] + offset_angle_;
                angle = std::max(0.0, std::min(angle, SERVO_MAX_ANGLE));
                if (debug_)
                {

                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "Setting angle %f for joint '%s'",
                                         angle, joint.name.c_str());
                }
                if (!setAngleToPwm(angle, pwm_map_[joint.name]))
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to set angle %f for joint '%s'",
                                 angle_map_[joint.name], joint.name.c_str());
                    return hardware_interface::return_type::ERROR;
                }
            }
            else if (joint.command_interfaces[0].name == "velocity")
            {
                // Throttle关节：根据归一化velocity计算PWM占空比
                double velocity = throttle_velocity_;
                velocity = std::max(0.0, std::min(velocity, 1.0));
                double min_percent = throttle_min_percentage_;
                double max_percent = throttle_max_percentage_;
                double duty_percentage = min_percent + (max_percent - min_percent) * velocity;
                unsigned int duty_ns = static_cast<unsigned int>(throttle_pwm_period_ns * (duty_percentage / 100.0));
                if (debug_)
                {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "Setting duty cycle %.2f%% (%d ns) for throttle joint '%s'", duty_percentage, duty_ns, joint.name.c_str());
                }
                if (!pwm_map_[joint.name]->setDutyCycle(duty_ns))
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to set duty cycle for throttle joint '%s'",
                                 joint.name.c_str());
                    return hardware_interface::return_type::ERROR;
                }
            }
        }

        return hardware_interface::return_type::OK;
    }
    hardware_interface::CallbackReturn DartFlySystemHardwareActuator::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        // 依次关闭所有PWM
        for (auto &joint : info_.joints)
        {
            if (!pwm_map_[joint.name]->disable())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to disable PWM for joint '%s'",
                    joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // 在线程中播放声音
        buzzer_thread = std::thread([]()
                                         { play_songs(buzzer_autopilot_disconnect); });

        buzzer_thread.join();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace dart_flysystem_hardware

// Export to pluginlib
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dart_flysystem_hardware::DartFlySystemHardwareActuator, hardware_interface::SystemInterface)
