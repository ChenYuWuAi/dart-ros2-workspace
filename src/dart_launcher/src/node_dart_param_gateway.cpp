#include "node_dart_param_gateway.hpp"
#include <fstream>

using json = nlohmann::json;

NodeDartParamGateway::NodeDartParamGateway()
    : rclcpp_lifecycle::LifecycleNode("node_dart_param_gateway")
{
    this->declare_parameter<std::string>("param_database_path", "/tmp");
}

NodeDartParamGateway::~NodeDartParamGateway()
{
}

void NodeDartParamGateway::load_params_from_file()
{
    std::string database_path;
    this->get_parameter("param_database_path", database_path);

    std::ifstream dart_param_file(database_path + "/dart_param.json");
    std::ifstream dart_protocols_file(database_path + "/dart_protocols.json");

    if (dart_param_file.is_open())
    {
        json param_json;
        dart_param_file >> param_json;
        dart_param_file.close();
        current_dart_param_ = param_json.get<dart_msgs::msg::DartLauncherParams>();
        RCLCPP_INFO(get_logger(), "Loaded DartParams from file.");
    }

    if (dart_protocols_file.is_open())
    {
        json protocols_json;
        dart_protocols_file >> protocols_json;
        dart_protocols_file.close();
        target_dart_param_ = protocols_json.get<dart_msgs::msg::DartLauncherParams>();
        RCLCPP_INFO(get_logger(), "Loaded DartProtocols from file.");
    }
}

void NodeDartParamGateway::save_params_to_file()
{
    std::string database_path;
    this->get_parameter("param_database_path", database_path);

    std::ofstream dart_param_file(database_path + "/dart_param.json");
    std::ofstream dart_protocols_file(database_path + "/dart_protocols.json");

    if (dart_param_file.is_open())
    {
        json param_json = current_dart_param_;
        dart_param_file << param_json.dump(4);
        dart_param_file.close();
        RCLCPP_INFO(get_logger(), "Saved DartParams to file.");
    }

    if (dart_protocols_file.is_open())
    {
        json protocols_json = target_dart_param_;
        dart_protocols_file << protocols_json.dump(4);
        dart_protocols_file.close();
        RCLCPP_INFO(get_logger(), "Saved DartProtocols to file.");
    }
}

void NodeDartParamGateway::start_daemon()
{
    daemon_thread_ = std::thread([this]()
    {
        while (rclcpp::ok())
        {
            if (current_dart_param_.last_param_update_time == 0)
            {
                RCLCPP_WARN(get_logger(), "Detected MCU restart, resynchronizing parameters...");
                this->save_params_to_file();
            }

            if (current_dart_param_ != target_dart_param_)
            {
                RCLCPP_INFO(get_logger(), "Detected parameter mismatch, resynchronizing...");
                this->save_params_to_file();
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    daemon_thread_.detach();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_configure");
    load_params_from_file();
    start_daemon();
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

    save_params_to_file();
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

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_deactivate");
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
    auto node = std::make_shared<NodeDartParamGateway>();
    RCLCPP_INFO(node->get_logger(), "Node started. Spinning...");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}