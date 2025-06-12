// filepath: /home/ubuntu/dart-ros2-workspace/src/dart_launcher/src/node_dart_logger.cpp
#include "node_dart_logger.hpp"
#include <memory>

NodeDartLogger::NodeDartLogger(const rclcpp::NodeOptions &options)
    : LifecycleNode("node_dart_logger", options)
{
  RCLCPP_INFO(get_logger(), "正在初始化飞镖日志节点");
}

NodeDartLogger::~NodeDartLogger()
{
  RCLCPP_INFO(get_logger(), "飞镖日志节点已销毁");
}

NodeDartLogger::CallbackReturn
NodeDartLogger::on_configure(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "正在配置飞镖日志节点 [%s]", state.label().c_str());

  // 创建订阅器，但在激活之前不会接收消息
  // BestEffort
  status_sub_ = this->create_subscription<dart_msgs::msg::DartLauncherStatus>(
      "/dart_launcher_mcu/status", rclcpp::QoS(10).durability_volatile().best_effort(),
      std::bind(&NodeDartLogger::status_callback, this, std::placeholders::_1));

  // Reliable
  log_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/dart_launcher_mcu/log", 10,
      std::bind(&NodeDartLogger::log_callback, this, std::placeholders::_1));

  last_status_msg_ = std::make_unique<dart_msgs::msg::DartLauncherStatus>();

  return CallbackReturn::SUCCESS;
}

NodeDartLogger::CallbackReturn
NodeDartLogger::on_activate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "正在激活飞镖日志节点 [%s]", state.label().c_str());

  // 激活后将开始接收消息
  return CallbackReturn::SUCCESS;
}

NodeDartLogger::CallbackReturn
NodeDartLogger::on_deactivate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "正在停用飞镖日志节点 [%s]", state.label().c_str());
  return CallbackReturn::SUCCESS;
}

NodeDartLogger::CallbackReturn
NodeDartLogger::on_cleanup(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "正在清理飞镖日志节点 [%s]", state.label().c_str());

  // 清理订阅器
  status_sub_.reset();
  log_sub_.reset();
  last_status_msg_.reset();

  return CallbackReturn::SUCCESS;
}

NodeDartLogger::CallbackReturn
NodeDartLogger::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "正在关闭飞镖日志节点 [%s]", state.label().c_str());
  return CallbackReturn::SUCCESS;
}

void NodeDartLogger::status_callback(const dart_msgs::msg::DartLauncherStatus::SharedPtr msg)
{
  if (!last_status_msg_)
  {
    last_status_msg_ = std::make_unique<dart_msgs::msg::DartLauncherStatus>();
    *last_status_msg_ = *msg;
    RCLCPP_INFO(get_logger(), "首次收到飞镖发射器状态消息");
    return;
  }

  // 检查监控字段是否有变化
  if (check_status_changes(last_status_msg_.get(), msg))
  {
    *last_status_msg_ = *msg;
  }
}

void NodeDartLogger::log_callback(const std_msgs::msg::String::SharedPtr msg)
{
  // 对于log消息，直接打印所有内容
  RCLCPP_INFO(get_logger(), "[MCU] %s", msg->data.c_str());
}

bool NodeDartLogger::check_status_changes(
    const dart_msgs::msg::DartLauncherStatus *last_msg,
    const dart_msgs::msg::DartLauncherStatus::SharedPtr msg)
{
  bool has_changes = false;

  // 检查需要监控的字段是否有变化

  // 检查primary_yaw_offset
  if (last_msg->primary_yaw_offset != msg->primary_yaw_offset)
  {
    RCLCPP_INFO(get_logger(), "primary_yaw_offset变更: %d -> %d",
                last_msg->primary_yaw_offset, msg->primary_yaw_offset);
    has_changes = true;
  }

  // 检查电机在线状态
  if (last_msg->motor_yaw_online != msg->motor_yaw_online)
  {
    RCLCPP_INFO(get_logger(), "motor_yaw_online变更: %s -> %s",
                last_msg->motor_yaw_online ? "在线" : "离线",
                msg->motor_yaw_online ? "在线" : "离线");
    has_changes = true;
  }

  // 检查装载电机在线状态
  for (size_t i = 0; i < 2; i++)
  {
    if (last_msg->motor_loader_online[i] != msg->motor_loader_online[i])
    {
      RCLCPP_INFO(get_logger(), "motor_loader_online[%zu]变更: %s -> %s", i,
                  last_msg->motor_loader_online[i] ? "在线" : "离线",
                  msg->motor_loader_online[i] ? "在线" : "离线");
      has_changes = true;
    }
  }

  // 检查触发器电机在线状态
  if (last_msg->motor_trigger_online != msg->motor_trigger_online)
  {
    RCLCPP_INFO(get_logger(), "motor_trigger_online变更: %s -> %s",
                last_msg->motor_trigger_online ? "在线" : "离线",
                msg->motor_trigger_online ? "在线" : "离线");
    has_changes = true;
  }

  // 检查裁判系统在线状态
  if (last_msg->judge_online != msg->judge_online)
  {
    RCLCPP_INFO(get_logger(), "judge_online变更: %s -> %s",
                last_msg->judge_online ? "在线" : "离线",
                msg->judge_online ? "在线" : "离线");
    has_changes = true;
  }

  // 检查遥控器在线状态
  if (last_msg->rc_online != msg->rc_online)
  {
    RCLCPP_INFO(get_logger(), "rc_online变更: %s -> %s",
                last_msg->rc_online ? "在线" : "离线",
                msg->rc_online ? "在线" : "离线");
    has_changes = true;
  }

  // 检查飞镖状态
  if (last_msg->dart_state != msg->dart_state)
  {
    RCLCPP_INFO(get_logger(), "dart_state变更: %d -> %d",
                last_msg->dart_state, msg->dart_state);
    has_changes = true;
  }

  // 检查飞镖发射进程
  if (last_msg->dart_launch_process != msg->dart_launch_process)
  {
    RCLCPP_INFO(get_logger(), "dart_launch_process变更: %d -> %d",
                last_msg->dart_launch_process, msg->dart_launch_process);
    has_changes = true;
  }

  // 检查最后发射速度
  if (last_msg->last_launch_speed != msg->last_launch_speed)
  {
    RCLCPP_INFO(get_logger(), "last_launch_speed变更: %.2f -> %.2f",
                last_msg->last_launch_speed, msg->last_launch_speed);
    has_changes = true;
  }

  // 检查最后发射时间
  if (last_msg->last_launch_time != msg->last_launch_time)
  {
    RCLCPP_INFO(get_logger(), "last_launch_time变更: %lu -> %lu",
                last_msg->last_launch_time, msg->last_launch_time);
    has_changes = true;
  }

  // 检查飞镖发射口状态
  if (last_msg->dart_launch_opening_status != msg->dart_launch_opening_status)
  {
    RCLCPP_INFO(get_logger(), "dart_launch_opening_status变更: %d -> %d",
                last_msg->dart_launch_opening_status, msg->dart_launch_opening_status);
    has_changes = true;
  }

  // 检查比赛进程
  if (last_msg->game_progress != msg->game_progress)
  {
    RCLCPP_INFO(get_logger(), "game_progress变更: %d -> %d",
                last_msg->game_progress, msg->game_progress);
    has_changes = true;
  }

  // 检查飞镖剩余时间
  if (last_msg->dart_remaining_time != msg->dart_remaining_time)
  {
    RCLCPP_INFO(get_logger(), "dart_remaining_time变更: %d -> %d",
                last_msg->dart_remaining_time, msg->dart_remaining_time);
    has_changes = true;
  }

  // 检查最新发射命令时间
  if (last_msg->latest_launch_cmd_time != msg->latest_launch_cmd_time)
  {
    RCLCPP_INFO(get_logger(), "latest_launch_cmd_time变更: %d -> %d",
                last_msg->latest_launch_cmd_time, msg->latest_launch_cmd_time);
    has_changes = true;
  }

  // 检查阶段剩余时间
  if (last_msg->stage_remain_time != msg->stage_remain_time)
  {
    RCLCPP_INFO(get_logger(), "stage_remain_time变更: %d -> %d",
                last_msg->stage_remain_time, msg->stage_remain_time);
    has_changes = true;
  }

  // 检查params参数
  if (last_msg->params.primary_yaw != msg->params.primary_yaw ||
      last_msg->params.primary_force != msg->params.primary_force ||
      last_msg->params.primary_force_offset != msg->params.primary_force_offset ||
      last_msg->params.dart_launch_process_offset_begin != msg->params.dart_launch_process_offset_begin ||
      last_msg->params.dart_launch_process_offset_end != msg->params.dart_launch_process_offset_end ||
      last_msg->params.auto_aim_enabled != msg->params.auto_aim_enabled ||
      last_msg->params.target_auto_aim_x_axis != msg->params.target_auto_aim_x_axis ||
      last_msg->params.last_param_update_time != msg->params.last_param_update_time)
  {
    RCLCPP_INFO(get_logger(), "params变更: primary_yaw=%d, primary_force=%d, primary_force_offset=%d, "
                              "dart_launch_process_offset_begin=%d, dart_launch_process_offset_end=%d, "
                              "auto_aim_enabled=%s, target_auto_aim_x_axis=%.2f, last_param_update_time=%lu",
                msg->params.primary_yaw, msg->params.primary_force, msg->params.primary_force_offset,
                msg->params.dart_launch_process_offset_begin, msg->params.dart_launch_process_offset_end,
                msg->params.auto_aim_enabled ? "启用" : "禁用", msg->params.target_auto_aim_x_axis,
                msg->params.last_param_update_time);
    has_changes = true;
  }

  // 检查辅助偏移量数组
  bool aux_yaw_changed = false;
  bool aux_force_changed = false;

  for (size_t i = 0; i < 4; i++)
  {
    if (last_msg->params.auxiliary_yaw_offsets[i] != msg->params.auxiliary_yaw_offsets[i])
    {
      aux_yaw_changed = true;
    }
    if (last_msg->params.auxiliary_force_offsets[i] != msg->params.auxiliary_force_offsets[i])
    {
      aux_force_changed = true;
    }
  }

  if (aux_yaw_changed)
  {
    RCLCPP_INFO(get_logger(), "params.auxiliary_yaw_offsets变更: [%d, %d, %d, %d]",
                msg->params.auxiliary_yaw_offsets[0], msg->params.auxiliary_yaw_offsets[1],
                msg->params.auxiliary_yaw_offsets[2], msg->params.auxiliary_yaw_offsets[3]);
    has_changes = true;
  }

  if (aux_force_changed)
  {
    RCLCPP_INFO(get_logger(), "params.auxiliary_force_offsets变更: [%d, %d, %d, %d]",
                msg->params.auxiliary_force_offsets[0], msg->params.auxiliary_force_offsets[1],
                msg->params.auxiliary_force_offsets[2], msg->params.auxiliary_force_offsets[3]);
    has_changes = true;
  }

  // 检查protocols参数（与params类似）
  if (last_msg->protocols.primary_yaw != msg->protocols.primary_yaw ||
      last_msg->protocols.primary_force != msg->protocols.primary_force ||
      last_msg->protocols.primary_force_offset != msg->protocols.primary_force_offset ||
      last_msg->protocols.dart_launch_process_offset_begin != msg->protocols.dart_launch_process_offset_begin ||
      last_msg->protocols.dart_launch_process_offset_end != msg->protocols.dart_launch_process_offset_end ||
      last_msg->protocols.auto_aim_enabled != msg->protocols.auto_aim_enabled ||
      last_msg->protocols.target_auto_aim_x_axis != msg->protocols.target_auto_aim_x_axis ||
      last_msg->protocols.last_param_update_time != msg->protocols.last_param_update_time)
  {
    RCLCPP_INFO(get_logger(), "protocols变更: primary_yaw=%d, primary_force=%d, primary_force_offset=%d, "
                              "dart_launch_process_offset_begin=%d, dart_launch_process_offset_end=%d, "
                              "auto_aim_enabled=%s, target_auto_aim_x_axis=%.2f, last_param_update_time=%lu",
                msg->protocols.primary_yaw, msg->protocols.primary_force, msg->protocols.primary_force_offset,
                msg->protocols.dart_launch_process_offset_begin, msg->protocols.dart_launch_process_offset_end,
                msg->protocols.auto_aim_enabled ? "启用" : "禁用", msg->protocols.target_auto_aim_x_axis,
                msg->protocols.last_param_update_time);
    has_changes = true;
  }

  // 检查protocols辅助偏移量数组
  bool protocols_aux_yaw_changed = false;
  bool protocols_aux_force_changed = false;

  for (size_t i = 0; i < 4; i++)
  {
    if (last_msg->protocols.auxiliary_yaw_offsets[i] != msg->protocols.auxiliary_yaw_offsets[i])
    {
      protocols_aux_yaw_changed = true;
    }
    if (last_msg->protocols.auxiliary_force_offsets[i] != msg->protocols.auxiliary_force_offsets[i])
    {
      protocols_aux_force_changed = true;
    }
  }

  if (protocols_aux_yaw_changed)
  {
    RCLCPP_INFO(get_logger(), "protocols.auxiliary_yaw_offsets变更: [%d, %d, %d, %d]",
                msg->protocols.auxiliary_yaw_offsets[0], msg->protocols.auxiliary_yaw_offsets[1],
                msg->protocols.auxiliary_yaw_offsets[2], msg->protocols.auxiliary_yaw_offsets[3]);
    has_changes = true;
  }

  if (protocols_aux_force_changed)
  {
    RCLCPP_INFO(get_logger(), "protocols.auxiliary_force_offsets变更: [%d, %d, %d, %d]",
                msg->protocols.auxiliary_force_offsets[0], msg->protocols.auxiliary_force_offsets[1],
                msg->protocols.auxiliary_force_offsets[2], msg->protocols.auxiliary_force_offsets[3]);
    has_changes = true;
  }

  if (has_changes)
  {
    std::string status_info = "总状态：";
    status_info += "舱门状态: " + std::to_string(msg->dart_launch_opening_status) + ", ";
    status_info += "比赛阶段剩余时间: " + std::to_string(msg->stage_remain_time) + ", ";
    status_info += "比赛进程: " + std::to_string(msg->game_progress) + ", ";
    status_info += "最新发射命令时间: " + std::to_string(msg->latest_launch_cmd_time) + ", ";
    status_info += "Yaw电机: " + std::string(msg->motor_yaw_online ? "在线" : "离线") + ", ";
    status_info += "Load装载电机: [" + std::to_string(msg->motor_loader_online[0]) + ", " +
                   std::to_string(msg->motor_loader_online[1]) + "], ";
    status_info += "Trigger电机: " + std::string(msg->motor_trigger_online ? "在线" : "离线") + ", ";
    status_info += "裁判系统: " + std::string(msg->judge_online ? "在线" : "离线") + ", ";
    status_info += "遥控器: " + std::string(msg->rc_online ? "在线" : "离线") + ", ";
    status_info += "发射架状态: " + std::to_string(msg->dart_state) + ", ";
    status_info += "发射进程: " + std::to_string(msg->dart_launch_process) + ", ";
    /// 电机相关角度Dump
    status_info += "Yaw角度: " + std::to_string(msg->motor_yaw_angle) + ", ";
    status_info += "Trigger角度: " + std::to_string(msg->motor_trigger_angle) + ", ";

    RCLCPP_INFO(get_logger(), "%s", status_info.c_str());
  }

  return has_changes;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().use_intra_process_comms(false);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<NodeDartLogger>(options);
  RCLCPP_INFO(node->get_logger(), "Node started. Spinning...");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}