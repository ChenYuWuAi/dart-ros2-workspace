#include "node_dart_param_gateway.hpp"
#include <fstream>

using json = nlohmann::json;

NodeDartParamGateway::NodeDartParamGateway(rclcpp::NodeOptions options)
    : rclcpp_lifecycle::LifecycleNode("node_dart_param_gateway", options)
{
}

NodeDartParamGateway::~NodeDartParamGateway()
{
}

bool NodeDartParamGateway::load_params_from_file(std::string database_path)
{
    std::ifstream dart_param_file(database_path + "/dart_param.json");
    std::ifstream dart_protocols_file(database_path + "/dart_protocols.json");

    if (dart_param_file.is_open())
    {
        try
        {
            json param_json;
            dart_param_file >> param_json;
            dart_param_file.close();
            current_dart_param_ = param_json.get<dart_msgs::msg::DartLauncherParams>();
            RCLCPP_INFO(get_logger(), "Loaded DartParams from file.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Failed to load DartParams from file: " << e.what());
            return false;
        }
    }
    else
        return false;

    if (dart_protocols_file.is_open())
    {
        try
        {
            json protocols_json;
            dart_protocols_file >> protocols_json;
            dart_protocols_file.close();
            target_dart_param_ = protocols_json.get<dart_msgs::msg::DartLauncherParams>();
            RCLCPP_INFO(get_logger(), "Loaded DartProtocols from file.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Failed to load DartProtocols from file: " << e.what());
            return false;
        }
    }
    else
        return false;

    return true;
}

bool NodeDartParamGateway::save_params_to_file(std::string database_path)
{
    std::ofstream dart_param_file(database_path + "/dart_param.json");
    std::ofstream dart_protocols_file(database_path + "/dart_protocols.json");

    if (dart_param_file.is_open())
    {
        try
        {
            json param_json = target_dart_param_;
            dart_param_file << param_json.dump(4); // Pretty print with 4 spaces
            dart_param_file.close();
            RCLCPP_INFO(get_logger(), "Saved DartParams to file.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Failed to save DartParams to file: " << e.what());
            return false;
        }
    }

    if (dart_protocols_file.is_open())
    {
        try
        {
            json protocols_json = target_dart_protocols_;
            dart_protocols_file << protocols_json.dump(4); // Pretty print with 4 spaces
            dart_protocols_file.close();
            RCLCPP_INFO(get_logger(), "Saved DartProtocols to file.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Failed to save DartProtocols to file: " << e.what());
            return false;
        }
    }
    else
        return false;
    return true;
}

void NodeDartParamGateway::start_daemon()
{
    daemon_thread_ = std::thread([this]()
                                 {
        RCLCPP_INFO(get_logger(), "Daemon thread started.");
        while (rclcpp::ok() && daemon_running_)
        {
            if (current_dart_param_.last_param_update_time == 0)
            {
                RCLCPP_WARN(get_logger(), "Detected MCU restart, resynchronizing parameters...");
                dart_param_pub_->publish(current_dart_param_);
                auto buzzer_msg = std_msgs::msg::Int32();
                buzzer_msg.data = BuzzerSound::BuzzerStartup; // Buzzer sound effect for MCU restart
                dart_buzzer_cmd_pub_->publish(buzzer_msg);
            }

            if (current_dart_param_ != target_dart_param_)
            {
                RCLCPP_INFO(get_logger(), "Detected parameter mismatch, resynchronizing...");
                dart_param_pub_->publish(target_dart_param_);
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        } });
    daemon_running_ = true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_configure");
    if (this->has_parameter("param_database_path"))
    {
        std::string database_path;
        this->get_parameter("param_database_path", database_path);
        // 检查路径是否存在，如果不存在则创建
        if (!std::filesystem::exists(database_path))
        {
            std::filesystem::create_directories(database_path);
            RCLCPP_INFO(get_logger(), "Created directory: %s", database_path.c_str());
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Directory already exists: %s", database_path.c_str());
        }

        if (!load_params_from_file(database_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load parameters from file, trying to rewrite default values.");
            defaultDartParams(target_dart_param_);
            defaultDartProtocols(target_dart_protocols_);
            if (save_params_to_file(database_path))
            {
                RCLCPP_INFO(get_logger(), "Default parameters saved to file.");
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Failed to save default parameters to file.");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Parameter 'param_database_path' not set.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void NodeDartParamGateway::process_qr_code(const std_msgs::msg::String::SharedPtr msg)
{
    json j = json::parse(msg->data);
    std::string command_type = j["command_type"];

    if (command_type == "DartParams")
    {
        current_dart_param_ = j["data"].get<dart_msgs::msg::DartLauncherParams>();
        RCLCPP_INFO(get_logger(), "Updated DartParams from QR code.");
    }
    else if (command_type == "DartProtocols")
    {
        target_dart_param_ = j["data"].get<dart_msgs::msg::DartLauncherParams>();
        RCLCPP_INFO(get_logger(), "Updated DartProtocols from QR code.");
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_activate");

    dart_param_pub_ = this->create_publisher<dart_msgs::msg::DartLauncherParams>(
        "/dart_launcher_mcu/cmd_params", rclcpp::QoS(10).durability_volatile().reliable());

    dart_buzzer_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "/dart_launcher_mcu/cmd_sound_effect", rclcpp::QoS(10).durability_volatile().reliable());

    dart_param_sub_ = this->create_subscription<dart_msgs::msg::DartLauncherParams>(
        "/dart_launcher_mcu/params", rclcpp::QoS(10).durability_volatile().best_effort(),
        [this](const dart_msgs::msg::DartLauncherParams::SharedPtr msg)
        {
            current_dart_param_ = *msg;
        });

    dart_qr_param_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/dart_launcher_detector/results/qrcode", rclcpp::QoS(10).durability_volatile().best_effort(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            process_qr_code(msg);
        });

    start_daemon();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_deactivate");
    dart_param_pub_.reset();
    dart_buzzer_cmd_pub_.reset();
    dart_param_sub_.reset();
    dart_qr_param_sub_.reset();
    if (daemon_thread_.joinable())
    {
        daemon_running_ = false;
        daemon_thread_.join();
        RCLCPP_INFO(get_logger(), "Daemon thread joined.");
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_cleanup");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_shutdown");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_error(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_error");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().use_intra_process_comms(false);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<NodeDartParamGateway>(options);
    RCLCPP_INFO(node->get_logger(), "Node started. Spinning...");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}