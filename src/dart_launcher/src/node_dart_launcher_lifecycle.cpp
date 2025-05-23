#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <csignal>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "dart_mcu/Drivers/stm32-buzzer/src/buzzer_examples.h"

// Empty service callback
#include "std_srvs/srv/trigger.hpp"

// 全局变量，用于信号处理
std::atomic<bool> g_signal_received{false};
std::shared_ptr<rclcpp::Node> g_lifecycle_manager_node;

// 信号处理函数
void signal_handler(int signum)
{
    g_signal_received.store(true);
    RCLCPP_INFO(g_lifecycle_manager_node->get_logger(), "Received signal %d, initiating shutdown...", signum);
}

using namespace std::chrono_literals;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using Trigger = std_srvs::srv::Trigger;

// SubNode for service calling
std::shared_ptr<rclcpp::Node> sub_node;
class SubNode : public rclcpp::Node
{
public:
    SubNode()
        : Node("sub_node")
    {
        this->declare_parameter<std::vector<std::string>>("nodes", {"node_dart_param_gateway",
                                                                    "node_dart_launcher_detector"});
        // Initialize the node
        RCLCPP_INFO(get_logger(), "SubNode initialized.");
    }
};

class LifecycleManager : public rclcpp::Node
{
public:
    LifecycleManager()
        : Node("lifecycle_manager")
    {
        // 注册信号处理函数
        std::signal(SIGUSR1, signal_handler); // 用户自定义信号1

        RCLCPP_INFO(get_logger(), "Signal handlers registered for SIGTERM, SIGINT, and SIGUSR1");

        // Load nodes from ROS parameters
        this->declare_parameter<std::vector<std::string>>("nodes", {"node_dart_param_gateway",
                                                                    "node_dart_launcher_detector"});
        nodes_ = this->get_parameter("nodes").as_string_array();

        for (const auto &node_name : nodes_)
        {
            RCLCPP_INFO(get_logger(), "Node to monitor registered: %s", node_name.c_str());
            node_states_[node_name] = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            offline_count_[node_name] = 0;
        }

        dart_buzzer_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/dart_launcher_mcu/cmd_sound_effect", rclcpp::QoS(10).durability_volatile().reliable());

        // Sequentially configure and activate on startup
        startup_timer_ = create_wall_timer(
            100ms, std::bind(&LifecycleManager::startup_sequence, this));

        // Periodic monitoring of node state
        monitor_timer_ = create_wall_timer(
            1s, std::bind(&LifecycleManager::monitor_nodes, this));

        // Shutdown service
        shutdown_srv_ = this->create_service<Trigger>(
            "shutdown_all",
            std::bind(&LifecycleManager::shutdown_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    bool having_shutdown = false;
    void shutdown_all()
    {
        if (having_shutdown)
            return;
        for (const auto &node_name : nodes_)
        {
            try
            {
                if (node_states_[node_name] == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                    change_each(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
                else if (node_states_[node_name] == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
                    change_each(node_name, lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
                else if (node_states_[node_name] == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
                    change_each(node_name, lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
                else
                {
                    RCLCPP_WARN(get_logger(), "%s is not in a valid state for shutdown", node_name.c_str());
                    continue;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Failed to transition %s: %s", node_name.c_str(), e.what());
                continue;
            }
        }
        having_shutdown = true;
    }

    void shutdown_callback(
        const std::shared_ptr<Trigger::Request> /*req*/,
        std::shared_ptr<Trigger::Response> res)
    {
        RCLCPP_INFO(get_logger(), "Shutdown service called, transitioning nodes to shutdown...");

        shutdown_all();

        res->success = true;
        res->message = "All nodes shutdown transitions triggered.";
        RCLCPP_INFO(get_logger(), "All nodes shutdown transitions triggered.");

        // Play shutdown sound
        buzzer_sound_effect(BuzzerSound::BuzzerWin10Remove);

        // Shutdown the node
        RCLCPP_INFO(get_logger(), "Shutting down lifecycle manager...");
        monitor_timer_->cancel();
    }

    void buzzer_sound_effect(int sound)
    {
        auto buzzer_msg = std_msgs::msg::Int32();
        buzzer_msg.data = sound;
        dart_buzzer_cmd_pub_->publish(buzzer_msg);
    }

    void startup_sequence()
    {
        startup_timer_->cancel();
        for (auto &p : nodes_)
        {
            if (!change_each(p, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
            {
                RCLCPP_ERROR(get_logger(), "Failed to configure %s", p.c_str());
                node_states_[p] = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
                return;
            }
            node_states_[p] = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
        }
        for (auto &p : nodes_)
        {
            if (!change_each(p, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
            {
                RCLCPP_ERROR(get_logger(), "Failed to activate %s", p.c_str());
                return;
            }
            node_states_[p] = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        }
        RCLCPP_INFO(get_logger(), "Startup sequence complete.");
    }

    bool change_each(const std::string &node_name, uint8_t transition)
    {
        auto configure_client = sub_node->create_client<ChangeState>(node_name + "/change_state");
        auto get_state_client = sub_node->create_client<GetState>(node_name + "/get_state");

        while (!configure_client->wait_for_service(10s))
        {
            static uint8_t retry_count = 0;
            RCLCPP_INFO(get_logger(), "Waiting for %s configure service...", node_name.c_str());
            std::this_thread::sleep_for(1s);
            retry_count++;
            if (retry_count > 3)
            {
                RCLCPP_ERROR(get_logger(), "Failed to connect to %s configure service", node_name.c_str());
                return false;
            }
        }

        // Configure
        if (!change_state_request(configure_client, transition))
        {
            return false;
        }

        RCLCPP_INFO(get_logger(), "%s transitioned to %u", node_name.c_str(), transition);
        return true;
    }

    void error_handler()
    {
        monitor_timer_->cancel();
        startup_timer_->cancel();
        RCLCPP_ERROR(get_logger(), "Error occurred, shutting down.");
        auto buzzer_msg = std_msgs::msg::Int32();
        buzzer_msg.data = BuzzerSound::BuzzerError;
        dart_buzzer_cmd_pub_->publish(buzzer_msg); // error sound
        // Shutdown all nodes
        shutdown_all();
        std::thread([&]()
                    {
            std::this_thread::sleep_for(5s);
            RCLCPP_INFO(get_logger(), "Restarting dart_ros2_run.service");
            system("sudo systemctl restart dart_ros2_run.service");
            exit(0); })
            .detach();
    }

    void monitor_nodes()
    {
        // 检查是否收到信号
        if (g_signal_received.load())
        {
            RCLCPP_INFO(get_logger(), "Signal received, initiating shutdown sequence...");
            shutdown_all();
            RCLCPP_INFO(get_logger(), "All nodes shutdown transitions triggered.");

            // Play shutdown sound
            buzzer_sound_effect(BuzzerSound::BuzzerWin10Remove);

            // Shutdown the node
            RCLCPP_INFO(get_logger(), "Shutting down lifecycle manager...");
            monitor_timer_->cancel();
            return;
        }

        for (auto &p : nodes_)
        {
            auto &node_name = p;
            auto &count = offline_count_[node_name];
            auto get_state_client = sub_node->create_client<GetState>(node_name + "/get_state");

            if (!get_state_client->wait_for_service(1s))
            {
                count++;
                RCLCPP_WARN(get_logger(), "%s get_state unavailable. Offline count: %d", node_name.c_str(), count);
                if (count > 1)
                {
                    RCLCPP_ERROR(get_logger(), "%s is offline. Triggering restart error handler", node_name.c_str());
                    error_handler();
                }
                node_states_[node_name] = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
                continue;
            }

            auto req = std::make_shared<GetState::Request>();
            auto future = get_state_client->async_send_request(req);
            auto status = rclcpp::spin_until_future_complete(sub_node, future, 1s);
            if (status == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                node_states_[node_name] = response->current_state.id;
                if (response->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                {
                    RCLCPP_ERROR(get_logger(), "%s is not active, current state: %d", node_name.c_str(), response->current_state.id);
                    error_handler();
                }
                // 服务可用，重置该节点计数
                count = 0;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Failed to get state from %s", node_name.c_str());
                count++;
                if (count > 1)
                {
                    RCLCPP_ERROR(get_logger(), "%s is offline. Triggering restart error handler", node_name.c_str());
                    error_handler();
                }
                node_states_[node_name] = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            }
        }
    }

    bool change_state_request(
        const rclcpp::Client<ChangeState>::SharedPtr &client,
        uint8_t transition)
    {
        auto request = std::make_shared<ChangeState::Request>();
        request->transition.id = transition;
        auto future = client->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(
            sub_node, future, 10s);
        if (status == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Transition %d sent to %s", transition, client->get_service_name());
            return true;
        }
        else
        {
            return false;
        }
    }

    std::shared_ptr<rclcpp::TimerBase> startup_timer_;
    std::shared_ptr<rclcpp::TimerBase> monitor_timer_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dart_buzzer_cmd_pub_;
    std::vector<std::string> nodes_;
    std::unordered_map<std::string, uint8_t> node_states_;
    std::unordered_map<std::string, uint8_t> offline_count_;
    rclcpp::Service<Trigger>::SharedPtr shutdown_srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto manager = std::make_shared<LifecycleManager>();
    g_lifecycle_manager_node = manager; // Assign after manager is created
    sub_node = std::make_shared<SubNode>();
    rclcpp::spin(manager);
    rclcpp::shutdown();
    return 0;
}
