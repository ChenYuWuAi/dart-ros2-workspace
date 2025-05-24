#include "node_dart_app.hpp"

#include <thread>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using namespace std;

// 全局变量
lv_ui guider_ui;
shared_ptr<NodeDartApp> g_node;

// 构造函数
NodeDartApp::NodeDartApp(const std::string &node_name, bool intra_process_comms)
    : rclcpp_lifecycle::LifecycleNode(node_name,
                                      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  RCLCPP_INFO(get_logger(), "Lifecycle node [%s] started.", node_name.c_str());

  // 声明参数

  // 启动LVGL UI界面
  lv_init();

  /*Linux display device init*/
  lv_linux_disp_init();

  lv_linux_input_create();

  setup_ui(&guider_ui);
  custom_init(&guider_ui);

  // Init mutex

  // 设置UI Loader到33%
  lv_label_set_text(guider_ui.scrLoader_labelLoader, "0%");
  lv_arc_set_value(guider_ui.scrLoader_arcLoader, 0);
  lv_label_set_text(guider_ui.scrLoader_labelLoadStage, "等待配置...");

  // 启动LVGL线程
  std::thread lvgl_thread(std::bind(&NodeDartApp::timer_callback_ui, this));
  lvgl_thread.detach();
}

// 析构函数
NodeDartApp::~NodeDartApp()
{
  RCLCPP_INFO(get_logger(), "Lifecycle node [%s] destroyed.", get_name());
}

// 配置回调
CallbackReturn NodeDartApp::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  with_ui_lock(g_node, [&]()
               {
    // 设置UI Loader到33%
    lv_label_set_text(guider_ui.scrLoader_labelLoader, "50%");
    lv_arc_set_value(guider_ui.scrLoader_arcLoader, 50);
    lv_label_set_text(guider_ui.scrLoader_labelLoadStage, "配置节点..."); });

  // 创建订阅者
  cv_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/dart/cv_image", 10,
      std::bind(&NodeDartApp::update_cv_image, this, std::placeholders::_1));

  dart_launcher_status_sub_ = this->create_subscription<dart_msgs::msg::DartLauncherStatus>(
      "/dart_launcher_mcu/status", rclcpp::QoS(10).durability_volatile().best_effort(),
      [this](const dart_msgs::msg::DartLauncherStatus::SharedPtr msg)
      {
        last_status_time_ = std::chrono::steady_clock::now();
        mcu_online_ = true;
        dart_launcher_status_ = *msg;
      });

  // 初始化定时器
  auto timer_callback_ip_update = [this]() -> void
  {
    update_ip_address();
  };

  timers_.push_back(this->create_wall_timer(1000ms, timer_callback_ip_update));

  return CallbackReturn::SUCCESS;
}

void NodeDartApp::screen_main_loop()
{
  // 主循环逻辑
  // 如果在scrHome
  static lv_obj_t *last_obj = nullptr;
  if (lv_scr_act() == guider_ui.scrHome)
  {
    // 更新UI
    with_ui_lock(g_node, [&]()
                 {
      auto now = std::chrono::system_clock::now();
      std::time_t now_c = std::chrono::system_clock::to_time_t(now);
      std::tm local_tm;
      localtime_r(&now_c, &local_tm);
      char datetime_buf[32];
      std::strftime(datetime_buf, sizeof(datetime_buf), "%Y-%m-%d %H:%M", &local_tm);
      lv_label_set_text(guider_ui.scrHome_labelDate, datetime_buf);

      auto now_steady = std::chrono::steady_clock::now();
      // 更新MCU工作状态
      if (std::chrono::duration_cast<std::chrono::seconds>(now_steady - last_status_time_).count() > 1)
      {
        mcu_online_ = false;
      }

      std::string prompt = "";
      static std::string last_prompt = "";
      bool error_ = false;

      if (mcu_online_ == false)
      {
        lv_label_set_text(guider_ui.scrHome_labelMCUMode, "未知");
        prompt += "MCU";
        error_ = true;
      }
      else
      {
        switch (dart_launcher_status_.dart_state)
        {
        case 100:
          lv_label_set_text(guider_ui.scrHome_labelMCUMode, "复位模式");
          break;
        case 101:
          lv_label_set_text(guider_ui.scrHome_labelMCUMode, "保护模式");
          break;
        case 102:
          lv_label_set_text(guider_ui.scrHome_labelMCUMode, "遥控模式");
          break;
        case 103:
        case 104:
        case 105:
        case 106:
          lv_label_set_text(guider_ui.scrHome_labelMCUMode, "比赛模式");
          break;
        case 107:
          lv_label_set_text(guider_ui.scrHome_labelMCUMode, "比赛保护");
          break;
        }

        // 检查是否有错误，yaw_online trigger_online和judge_online
        if (dart_launcher_status_.motor_yaw_online == false)
        {
          prompt += "Yaw电机 ";
          error_ = true;
        }
        if (dart_launcher_status_.motor_trigger_online == false)
        {
          prompt += "Trigger电机 ";
          error_ = true;
        }
        if (dart_launcher_status_.motor_loader_online[0] == false)
        {
          prompt += "Load0电机 ";
          error_ = true;
        }
        if (dart_launcher_status_.motor_loader_online[1] == false)
        {
          prompt += "Load1电机 ";
          error_ = true;
        }
        if (dart_launcher_status_.judge_online == false)
        {
          prompt += "裁判系统 ";
          error_ = true;
        }
        if (dart_launcher_status_.rc_online == false)
        {
          prompt += "遥控 ";
          error_ = true;
        }
      }

      // 检查是否需要生成报错信息
      if (!error_){
        if(prompt != last_prompt || last_obj != lv_scr_act())
          lv_label_set_text(guider_ui.scrHome_labelPrompt, "无异常");
        lv_obj_set_style_text_color(guider_ui.scrHome_labelPrompt, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(guider_ui.scrHome_contText, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(guider_ui.scrHome_labelPrompt, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
      }
      else
      {
        prompt += "离线";
        if(prompt != last_prompt || last_obj != lv_scr_act())
          lv_label_set_text(guider_ui.scrHome_labelPrompt, prompt.c_str());
        // 设置提示框颜色
        lv_obj_set_style_text_color(guider_ui.scrHome_labelPrompt, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(guider_ui.scrHome_contText, lv_color_hex(0xFF9800), LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(guider_ui.scrHome_labelPrompt, lv_color_hex(0xFF9800), LV_PART_MAIN|LV_STATE_DEFAULT);
      }
      last_prompt = prompt; });
  }

  else if (lv_scr_act() == guider_ui.scrParams)
  {
    // 更新参数界面，界面为表格格式tableDartStatus和tableDartParams
    // json
    // 初次进入时初始化表格格式
    if (last_obj != lv_scr_act())
    {
      // 初始化表格格式
      // 将dart_launcher_status_中的数据填入表格
      lv_table_set_col_cnt(guider_ui.scrParams_tableDartParams, 2);
      lv_table_set_row_cnt(guider_ui.scrParams_tableDartParams, 0);
      lv_table_set_col_width(guider_ui.scrParams_tableDartParams, 0, 150);
      lv_table_set_col_width(guider_ui.scrParams_tableDartParams, 1, 150);
    }
  last_obj = lv_scr_act();
}

// 激活回调
CallbackReturn NodeDartApp::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "正在激活...");

  // 加载初始数据
  update_ip_address();

  with_ui_lock(g_node, [&]()
               {
    // 设置UI Loader到100%
    lv_label_set_text(guider_ui.scrLoader_labelLoader, "100%");
    lv_label_set_text(guider_ui.scrLoader_labelLoadStage, "激活节点...");
    lv_arc_set_value(guider_ui.scrLoader_arcLoader, 100); });

  thread([&]()
         {
          this_thread::sleep_for(1s); // 等待1秒
    // 切换到主界面
    with_ui_lock(g_node, [&]()
    {
      setup_scr_scrHome(&guider_ui);
      lv_screen_load(guider_ui.scrHome);
    });
    timers_.push_back(this->create_wall_timer(100ms, std::bind(&NodeDartApp::screen_main_loop, this))); })
      .detach();

  return CallbackReturn::SUCCESS;
}

// 停用回调
CallbackReturn NodeDartApp::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "正在停用...");

  return CallbackReturn::SUCCESS;
}

// 清理回调
CallbackReturn NodeDartApp::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "正在清理...");

  // 清除订阅者
  dart_launcher_status_sub_.reset();
  green_light_sub_.reset();
  cv_image_sub_.reset();

  // 清除定时器
  timers_.clear();

  return CallbackReturn::SUCCESS;
}

// 关闭回调
CallbackReturn NodeDartApp::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "正在关闭...");

  // 等待UI线程完成
  if (ui_thread_ && ui_thread_->joinable())
  {
    ui_thread_->join();
  }

  // 等待文件监视线程完成
  if (watch_file_thread_ && watch_file_thread_->joinable())
  {
    watch_file_thread_->join();
  }

  // 清理LVGL资源
  mutex_ui_.lock();
  lv_obj_clean(lv_scr_act()); // 清理活动屏幕
  lv_timer_handler();         // 调用定时器处理程序
  lv_deinit();                // 反初始化LVGL
  mutex_ui_.unlock();

  return CallbackReturn::SUCCESS;
}

// 错误回调
CallbackReturn NodeDartApp::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "发生错误!");
  return CallbackReturn::SUCCESS;
}

// UI线程回调
void NodeDartApp::timer_callback_ui()
{
  try
  {
    while (rclcpp::ok())
    {
      this->mutex_ui_.lock(); // 锁定互斥锁，防止在清理屏幕时更新GUI
      lv_timer_handler();
      this->mutex_ui_.unlock();
      this_thread::sleep_for(5ms);
    }
    this->mutex_ui_.lock();
    lv_obj_clean(lv_scr_act()); // 清理活动屏幕
    lv_timer_handler();         // 调用定时器处理程序
    lv_deinit();                // 反初始化LVGL
    this->mutex_ui_.unlock();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "UI线程异常: %s", e.what());
    if (!rclcpp::ok())
      return;
  }
}

// 更新IP地址
void NodeDartApp::update_ip_address()
{
  // 获取IP地址
  std::string interface = "wlan0";
  std::string ip = "127.0.0.1"; // 这里需要实现getIPAddress函数，或者直接使用系统命令获取

  if (!mutex_ui_.try_lock())
    return;

  RCLCPP_INFO_ONCE(this->get_logger(), "IP地址: %s", ip.c_str());
  // TODO: 更新GUI中的IP地址
  // lv_label_set_text(guider_ui., ip.c_str());
  mutex_ui_.unlock();
}

// 更新CV图像
void NodeDartApp::update_cv_image(sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  // 更新图像
  if (!mutex_ui_.try_lock())
    return;
  {
    // 此处为占位符，实际实现需要根据原有代码移植
    if (msg == nullptr)
      return;

    RCLCPP_DEBUG(this->get_logger(), "更新CV图像");
  }
}

// 主函数
int main(int argc, char **argv)
{
  // 初始化ROS
  rclcpp::init(argc, argv);

  // 创建生命周期节点
  g_node = std::make_shared<NodeDartApp>("node_dart_app");
  RCLCPP_INFO(g_node->get_logger(), "Node started. Spinning...");
  rclcpp::spin(g_node->get_node_base_interface());

  // 清理资源
  rclcpp::shutdown();

  return 0;
}
