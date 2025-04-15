/**
    本文件用于维护飞镖发射参数和飞镖信息服务。
 */
#include <rclcpp/rclcpp.hpp>
// qos
#include <info/msg/dart_param.hpp>
#include <node_dart_config.hpp>
#include <info/msg/dart_param.hpp>
#include <dart_config.hpp>

#include <thread>
#include <fstream>
#include <iostream>

#ifndef YAML_PATH
#define YAML_PATH "/home/chenyu/dart24_ws/install/dart_controller/share/dart_controller/config" + "dart_config.yaml"
#endif

namespace fs = std::filesystem;
using namespace DartConfig;

NodeDartConfig::NodeDartConfig(const std::string &yaml_file)
    : Node("dart_config"), yaml_file_(yaml_file)
{
    declareParameters(*this);
    loadParameters();
    // syncParameters(false);
    watchFile();

    auto sub_dart_param_change_callback = [this](const info::msg::DartParam::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received dart param: %s", msg->header.frame_id.c_str());
        loadParametersfromMsg(*this, msg);
        // 保存参数
        saveParameters();
    };

    sub_dart_param_[0] = this->create_subscription<info::msg::DartParam>(
        "/dart_controller/dart_launcher_param_cmd",
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(), sub_dart_param_change_callback);

    sub_dart_param_[1] = this->create_subscription<info::msg::DartParam>(
        "/dart_controller/dart_launcher_present_param",
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(), sub_dart_param_change_callback);

    // 重设参数服务
    srv_reset_ = this->create_service<std_srvs::srv::Empty>(
        "/dart_config/reset_all_param_to_default",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
               std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Resetting parameters to default...");
            yaml_file_ += ".default";
            loadParameters();
            yaml_file_.erase(yaml_file_.end() - 8, yaml_file_.end()); // remove ".default"
            saveParameters();
            // 让watchFile工作一次
            last_write_time = fs::file_time_type::min();
            //  response
            response->structure_needs_at_least_one_member = 0;
        });

    // 重新同步参数服务
    srv_resync_ = this->create_service<std_srvs::srv::Empty>(
        "/dart_config/resync_all_param",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
               std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Resynchronizing parameters...");
            // 异步执行器，同步参数
            std::thread([this]()
                        { syncParameters(true); })
                .detach();
            //  response
            response->structure_needs_at_least_one_member = 0;
        });
}

void NodeDartConfig::loadParameters()
{
    YAML::Node config;
    try
    {
        RCLCPP_INFO(this->get_logger(), "Loading parameters from %s", yaml_file_.c_str());
        config = YAML::LoadFile(yaml_file_);
        config = config["/dart_config"]["ros__parameters"];
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load parameters from %s", yaml_file_.c_str());
        config = YAML::Node();
    }

    // If the file is empty (or corrupted), generate default parameters
    if (config.IsNull() || config.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "YAML file is empty or corrupted, generating default parameters...");
        saveParameters();
        return;
    }

    for (const auto &it : config)
    {
        const std::string &param_name = it.first.as<std::string>();
        
        const auto &value = it.second;

        if (value.IsScalar())
        {
            auto param_type = this->get_parameter(param_name).get_type();
            // skip qos_overrides.****, start_type_description_service, use_sim_time
            if (param_name.find("qos_overrides") != std::string::npos || param_name == "start_type_description_service" || param_name == "use_sim_time")
            {
                continue;
            }
            switch (param_type)
            {
            case rclcpp::ParameterType::PARAMETER_BOOL:
                this->set_parameter(rclcpp::Parameter(param_name, value.as<bool>()));
                break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                this->set_parameter(rclcpp::Parameter(param_name, value.as<int32_t>()));
                break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
                this->set_parameter(rclcpp::Parameter(param_name, value.as<double>()));
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unsupported parameter type for %s", param_name.c_str());
                break;
            }
        }
        else if (value.IsSequence())
        {
            if (this->get_parameter(param_name).get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
            {
                std::vector<std::string> string_array;
                for (const auto &element : value)
                {
                    string_array.push_back(element.as<std::string>());
                }
                this->set_parameter(rclcpp::Parameter(param_name, string_array));
            }
            else if (this->get_parameter(param_name).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
            {
                std::vector<int> int_array;
                for (const auto &element : value)
                {
                    int_array.push_back(element.as<int32_t>());
                }
                this->set_parameter(rclcpp::Parameter(param_name, int_array));
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unsupported parameter type for %s", param_name.c_str());
        }
    }
    RCLCPP_INFO(this->get_logger(), "Parameters loaded from %s", yaml_file_.c_str());
}

void NodeDartConfig::saveParameters()
{
    YAML::Node main;
    YAML::Node parameters;
    YAML::Node config;

    auto param_list = this->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
    for (const auto &param : param_list.names)
    {
        if (param.find("qos_overrides") != std::string::npos || param == "start_type_description_service" || param == "use_sim_time")
        {
            continue;
        }
        if (!(this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) && !(this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY))
        {
            auto param_value = this->get_parameter(param).value_to_string();
            config[param] = param_value;
            continue;
        }
        else if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            auto param_value = this->get_parameter(param).as_integer_array();
            YAML::Node int_array;
            for (const auto &element : param_value)
            {
                int_array.push_back(element);
            }
            config[param] = int_array;
        }
        else if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            auto param_value = this->get_parameter(param).as_string_array();
            YAML::Node string_array;
            for (const auto &element : param_value)
            {
                string_array.push_back(element);
            }
            config[param] = string_array;
        }
    }

    std::ofstream fout(yaml_file_);
    parameters["ros__parameters"] = config;
    main["/dart_config"] = parameters;
    fout << main;
    fout.close();
    last_write_time = fs::last_write_time(yaml_file_);
    RCLCPP_INFO(this->get_logger(), "Parameters saved to %s", yaml_file_.c_str());
}

void NodeDartConfig::syncParameters(rclcpp::Parameter &param, bool spinning)
{
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(param.to_parameter_msg());

    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for service to appear...");
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting...");
            return;
        }
    }

    auto result = client_->async_send_request(request);
    if (spinning) // 若spinning，说明节点已经由执行器运行，直接调用服务
    {
        result.wait();
        if (result.valid() && result.get()->results[0].successful)
        {
            RCLCPP_INFO(this->get_logger(), "Parameter %s synchronized", param.get_name().c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to synchronize parameter %s", param.get_name().c_str());
        }
    }
    else // 若不spinning，说明节点还未由执行器运行，需要等待执行器运行后再调用服务
    {
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Parameter %s synchronized", param.get_name().c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to synchronize parameter %s", param.get_name().c_str());
        }
    }
}

void NodeDartConfig::syncParameters(bool spinning)
{
    auto param_list = this->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
    std::vector<std::string> nodes = {"/can_agent/set_parameters", "/lvgl_ui/set_parameters", "/dart_logger_dog/set_parameters"};
    for (size_t i = 0; i < 3; i++)
    {

        RCLCPP_INFO(this->get_logger(), "Synchronizing parameters to %s ...", nodes[i].c_str());
        client_ = this->create_client<rcl_interfaces::srv::SetParameters>(nodes[i]);
        for (const auto &param : param_list.names)
        {
            // skip qos_overrides.****, start_type_description_service, use_sim_time
            if (param.find("qos_overrides") != std::string::npos || param == "start_type_description_service" || param == "use_sim_time")
            {
                continue;
            }

            auto param_value = this->get_parameter(param);
            syncParameters(param_value, spinning);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Parameters synchronized");
}

void NodeDartConfig::watchFile()
{
    last_write_time = fs::last_write_time(yaml_file_);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]()
        {
            auto current_write_time = fs::last_write_time(yaml_file_);

            if (current_write_time != last_write_time)
            {
                last_write_time = current_write_time;
                RCLCPP_INFO(this->get_logger(), "YAML file changed, reloading parameters...");
                loadParameters();
                // 调用异步执行器，同步参数
                std::thread([this]()
                            { syncParameters(true); })
                    .detach();
            }
        });
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NodeDartConfig>(std::string(YAML_PATH) + std::string("dart_config.yaml"));

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
