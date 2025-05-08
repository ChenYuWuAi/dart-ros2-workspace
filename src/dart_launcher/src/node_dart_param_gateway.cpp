#include "node_dart_param_gateway.hpp"
#include <fstream>
#include <filesystem>
#include <rclcpp/clock.hpp>

using json = nlohmann::json;

NodeDartParamGateway::NodeDartParamGateway(rclcpp::NodeOptions options)
    : rclcpp_lifecycle::LifecycleNode("node_dart_param_gateway", options),
      daemon_running_(false)
{
}

NodeDartParamGateway::~NodeDartParamGateway()
{
    // 确保守护进程已停止
    if (daemon_running_ && daemon_thread_.joinable())
    {
        daemon_running_ = false;
        daemon_thread_.join();
    }
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
            target_dart_param_ = param_json.get<dart_msgs::msg::DartLauncherParams>();
            dart_status_.params = target_dart_param_;
            RCLCPP_INFO(get_logger(), "Loaded DartParams from file.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Failed to load DartParams from file: " << e.what());
            return false;
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Failed to open DartParams file: %s/dart_param.json", database_path.c_str());
        return false;
    }

    if (dart_protocols_file.is_open())
    {
        try
        {
            json protocols_json;
            dart_protocols_file >> protocols_json;
            dart_protocols_file.close();
            target_dart_protocols_ = protocols_json.get<dart_msgs::msg::DartLauncherParams>();
            dart_status_.protocols = target_dart_protocols_;
            RCLCPP_INFO(get_logger(), "Loaded DartProtocols from file.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Failed to load DartProtocols from file: " << e.what());
            return false;
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Failed to open DartProtocols file: %s/dart_protocols.json", database_path.c_str());
        return false;
    }

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
            json param_json = dart_status_.params;
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
    else
    {
        RCLCPP_WARN(get_logger(), "Failed to open DartParams file for writing: %s/dart_param.json", database_path.c_str());
        return false;
    }

    if (dart_protocols_file.is_open())
    {
        try
        {
            json protocols_json = dart_status_.protocols;
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
    {
        RCLCPP_WARN(get_logger(), "Failed to open DartProtocols file for writing: %s/dart_protocols.json", database_path.c_str());
        return false;
    }
    return true;
}

void NodeDartParamGateway::start_daemon()
{
    // 确保之前的守护进程已经停止
    if (daemon_running_ && daemon_thread_.joinable())
    {
        daemon_running_ = false;
        daemon_thread_.join();
    }

    daemon_thread_ = std::thread([this]()
                                 {
        daemon_running_ = true;
        RCLCPP_INFO(get_logger(), "Daemon thread started.");

        enum class SyncState { IDLE, CHECK_MCU_RESTART, CHECK_MISMATCH, RESYNC, SAVE_TO_FILE };
        SyncState current_state = SyncState::IDLE;

        auto last_save_time = std::chrono::steady_clock::now();
        auto last_resync_time = std::chrono::steady_clock::now();
        std::string database_path;
        this->get_parameter("param_database_path", database_path);

        while (rclcpp::ok() && daemon_running_)
        {
            auto now_time = std::chrono::steady_clock::now();

            switch (current_state)
            {
            case SyncState::IDLE:
                if ((dart_status_.protocols.last_param_update_time == 0 || dart_status_.params.last_param_update_time == 0) && dart_status_.header.stamp.sec != 0)
                {
                    current_state = SyncState::CHECK_MCU_RESTART;
                    last_resync_time = now_time;
                }
                else if (dart_status_.params != target_dart_param_ || dart_status_.protocols != target_dart_protocols_)
                {
                    current_state = SyncState::CHECK_MISMATCH;
                }
                else if (std::chrono::duration_cast<std::chrono::seconds>(now_time - last_save_time).count() >= 30)
                {
                    current_state = SyncState::SAVE_TO_FILE;
                }
                break;

            case SyncState::CHECK_MCU_RESTART:
                RCLCPP_WARN(get_logger(), "Detected MCU restart, resynchronizing parameters and protocols...");
                dart_params_pub_->publish(target_dart_param_);
                dart_protocols_pub_->publish(target_dart_protocols_);
                
                // 在timeout时间内等待MCU参数更新
                if(now_time - last_resync_time < std::chrono::seconds(5) && (dart_status_.params != target_dart_param_ || dart_status_.protocols != target_dart_protocols_))
                {
                    RCLCPP_DEBUG(get_logger(), "Waiting for MCU parameters to update...");
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                else
                {
                    // 超时未更新，进入IDLE重新发布参数
                    current_state = SyncState::IDLE;
                    RCLCPP_WARN(get_logger(), "MCU parameters not updated within timeout, resynchronizing...");
                }
                break;
            case SyncState::CHECK_MISMATCH:
                if (dart_status_.params != target_dart_param_)
                {
                    RCLCPP_INFO(get_logger(), "Detected parameter mismatch, resynchronizing params...");
                    dart_params_pub_->publish(target_dart_param_);
                }

                if (dart_status_.protocols != target_dart_protocols_)
                {
                    RCLCPP_INFO(get_logger(), "Detected protocols mismatch, resynchronizing protocols...");
                    dart_params_pub_->publish(target_dart_protocols_);
                }

                last_resync_time = now_time;
                current_state = SyncState::IDLE;
                break;

            case SyncState::SAVE_TO_FILE:
                save_params_to_file(database_path);
                last_save_time = now_time;
                current_state = SyncState::IDLE;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(get_logger(), "Daemon thread stopped."); });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_configure");

    // 声明参数
    if (!this->has_parameter("param_database_path"))
        this->declare_parameter("param_database_path", "");

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
            defaultDartParams(dart_status_.params);
            defaultDartProtocols(dart_status_.protocols);
            target_dart_param_ = dart_status_.params;
            target_dart_protocols_ = dart_status_.protocols;
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

// 获取当前时间戳（ms）
static uint64_t get_ros_time_ms(const rclcpp::Clock &clock)
{
    auto now = clock.now();
    return static_cast<uint64_t>(now.seconds() * 1000.0);
}

void NodeDartParamGateway::process_qr_code(const std_msgs::msg::String::SharedPtr msg)
{
    try
    {
        json j = json::parse(msg->data);
        std::string command_type = j["command_type"];
        std::string database_path;
        this->get_parameter("param_database_path", database_path);
        auto now_ms = get_ros_time_ms(*this->get_clock());

        if (command_type == "DartParams")
        {
            // 先用原有target_dart_param_，只更新有的字段
            dart_msgs::msg::DartLauncherParams tmp = target_dart_param_;
            dart_msgs::msg::DartLauncherParams old = target_dart_param_;
            from_json(j["data"], tmp);
            tmp.last_param_update_time = now_ms;
            target_dart_param_ = tmp;
            dart_status_.params = tmp;
            RCLCPP_INFO(get_logger(), "Updated DartParams from QR code, set last_param_update_time=%lu", now_ms);
            save_params_to_file(database_path);
            if (dart_params_pub_)
                dart_params_pub_->publish(target_dart_param_);
        }
        else if (command_type == "DartProtocols")
        {
            dart_msgs::msg::DartLauncherParams tmp = target_dart_protocols_;
            from_json(j["data"], tmp);
            tmp.last_param_update_time = now_ms; // protocols本地更新时间
            target_dart_protocols_ = tmp;
            dart_status_.protocols = tmp;
            RCLCPP_INFO(get_logger(), "Updated DartProtocols from QR code, set last_param_update_time=%lu", now_ms);
            save_params_to_file(database_path);
            if (dart_params_pub_)
                dart_params_pub_->publish(target_dart_protocols_);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unknown command type: %s", command_type.c_str());
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Error parsing QR code data: " << e.what());
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_activate");

    dart_params_pub_ = this->create_publisher<dart_msgs::msg::DartLauncherParams>(
        "/dart_launcher_mcu/cmd_params", rclcpp::QoS(10).durability_volatile().reliable());

    dart_protocols_pub_ = this->create_publisher<dart_msgs::msg::DartLauncherParams>(
        "/dart_launcher_mcu/cmd_protocols", rclcpp::QoS(10).durability_volatile().reliable());

    dart_buzzer_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "/dart_launcher_mcu/cmd_sound_effect", rclcpp::QoS(10).durability_volatile().reliable());

    dart_param_sub_ = this->create_subscription<dart_msgs::msg::DartLauncherParams>(
        "/dart_launcher_mcu/params", rclcpp::QoS(10).durability_volatile().best_effort(),
        [this](const dart_msgs::msg::DartLauncherParams::SharedPtr msg)
        {
            dart_status_.params = *msg;
        });

    // 添加status订阅
    dart_status_sub_ = this->create_subscription<dart_msgs::msg::DartLauncherStatus>(
        "/dart_launcher_mcu/status", rclcpp::QoS(10).durability_volatile().best_effort(),
        [this](const dart_msgs::msg::DartLauncherStatus::SharedPtr msg)
        {
            // 判断下位机参数是否比本地新
            if (msg->params.last_param_update_time > target_dart_param_.last_param_update_time)
            {
                target_dart_param_ = msg->params;
                RCLCPP_INFO(get_logger(), "MCU参数更新，自动同步target_dart_param_，last_param_update_time=%lu", msg->params.last_param_update_time);
                std::string database_path;
                if (this->get_parameter("param_database_path", database_path))
                    save_params_to_file(database_path);
            }
            dart_status_ = *msg;
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

    // 在deactivate前保存参数
    std::string database_path;
    if (this->has_parameter("param_database_path") && this->get_parameter("param_database_path", database_path))
    {
        save_params_to_file(database_path);
    }

    // 关闭发布者和订阅者
    dart_params_pub_.reset();
    dart_protocols_pub_.reset();
    dart_buzzer_cmd_pub_.reset();
    dart_param_sub_.reset();
    dart_status_sub_.reset();
    dart_qr_param_sub_.reset();

    // 停止守护线程
    if (daemon_running_ && daemon_thread_.joinable())
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

    // 确保所有资源被释放
    if (dart_params_pub_)
        dart_params_pub_.reset();
    if (dart_protocols_pub_)

        if (dart_buzzer_cmd_pub_)
            dart_buzzer_cmd_pub_.reset();
    if (dart_param_sub_)
        dart_param_sub_.reset();
    if (dart_status_sub_)
        dart_status_sub_.reset();
    if (dart_qr_param_sub_)
        dart_qr_param_sub_.reset();

    // 确保守护线程已停止
    if (daemon_running_ && daemon_thread_.joinable())
    {
        daemon_running_ = false;
        daemon_thread_.join();
        RCLCPP_INFO(get_logger(), "Daemon thread joined during cleanup.");
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_shutdown");

    // 在节点关闭前保存参数
    std::string database_path;
    if (this->has_parameter("param_database_path") && this->get_parameter("param_database_path", database_path))
    {
        save_params_to_file(database_path);
    }

    // 确保守护线程已停止
    if (daemon_running_ && daemon_thread_.joinable())
    {
        daemon_running_ = false;
        daemon_thread_.join();
        RCLCPP_INFO(get_logger(), "Daemon thread joined during shutdown.");
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartParamGateway::on_error(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "NodeDartParamGateway on_error");

    // 尝试保存当前状态
    std::string database_path;
    if (this->has_parameter("param_database_path") && this->get_parameter("param_database_path", database_path))
    {
        save_params_to_file(database_path);
    }

    // 确保守护线程已停止
    if (daemon_running_ && daemon_thread_.joinable())
    {
        daemon_running_ = false;
        daemon_thread_.join();
        RCLCPP_INFO(get_logger(), "Daemon thread joined during error handling.");
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<NodeDartParamGateway>(options);
    RCLCPP_INFO(node->get_logger(), "Node started. Spinning...");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}