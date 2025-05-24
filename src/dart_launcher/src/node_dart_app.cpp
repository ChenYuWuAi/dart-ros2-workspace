#include "node_dart_app.hpp"

#include <thread>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include "dart_comm_share/include/dart_launcher_param.h"

using json = nlohmann::json;
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

  // 启动LVGL线程
  ui_thread_ = std::make_shared<std::thread>(std::bind(&NodeDartApp::timer_callback_ui, this));

  // 每5秒调用一次 update_network_status，避免阻塞主线程，使用异步线程执行
  timers_.push_back(
      this->create_wall_timer(
          5000ms,
          [this]()
          {
            std::thread([this]()
                        { this->update_network_status(); })
                .detach();
          }));
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
  greenlight_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/dart_launcher_detector/image/greenlight_processed", 10,
      std::bind(&NodeDartApp::update_greenlight_image, this, std::placeholders::_1));

  qrcode_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/dart_launcher_detector/image/qrcode", 10,
      std::bind(&NodeDartApp::update_qrcode_image, this, std::placeholders::_1));

  dart_launcher_status_sub_ = this->create_subscription<dart_msgs::msg::DartLauncherStatus>(
      "/dart_launcher_mcu/status", rclcpp::QoS(10).durability_volatile().best_effort(),
      [this](const dart_msgs::msg::DartLauncherStatus::SharedPtr msg)
      {
        last_status_time_ = std::chrono::steady_clock::now();
        mcu_online_ = true;
        dart_launcher_status_ = *msg;
      });

  return CallbackReturn::SUCCESS;
}

void NodeDartApp::screen_main_loop()
{
  // 主循环逻辑
  // 如果在scrHome
  static lv_obj_t *last_obj = nullptr;
  if (lv_scr_act() == guider_ui.scrHome)
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

    // 控制网络图标的可见性
    if (is_online_)
    {
      lv_obj_clear_flag(guider_ui.scrHome_imgIconWIFI, LV_OBJ_FLAG_HIDDEN); // 显示WiFi图标
      lv_obj_add_flag(guider_ui.scrHome_imgIconNoWIFI, LV_OBJ_FLAG_HIDDEN); // 隐藏无网络图标
    }
    else
    {
      lv_obj_add_flag(guider_ui.scrHome_imgIconWIFI, LV_OBJ_FLAG_HIDDEN); // 隐藏WiFi图标
      lv_obj_clear_flag(guider_ui.scrHome_imgIconNoWIFI, LV_OBJ_FLAG_HIDDEN);     // 显示无网络图标
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
    if (!error_)
    {
      if (prompt != last_prompt || last_obj != lv_scr_act())
        lv_label_set_text(guider_ui.scrHome_labelPrompt, "无异常");
      lv_obj_set_style_text_color(guider_ui.scrHome_labelPrompt, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(guider_ui.scrHome_contText, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(guider_ui.scrHome_labelPrompt, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
      prompt += "离线";
      if (prompt != last_prompt || last_obj != lv_scr_act())
        lv_label_set_text(guider_ui.scrHome_labelPrompt, prompt.c_str());
      // 设置提示框颜色
      lv_obj_set_style_text_color(guider_ui.scrHome_labelPrompt, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(guider_ui.scrHome_contText, lv_color_hex(0xFF9800), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_set_style_bg_color(guider_ui.scrHome_labelPrompt, lv_color_hex(0xFF9800), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    last_prompt = std::string(lv_label_get_text(guider_ui.scrHome_labelPrompt));
  }

  else if (lv_scr_act() == guider_ui.scrParams)
  {
    // 更新参数界面，界面为表格格式tableDartStatus和tableDartParams
    // 首次进入时初始化表格格式
    if (last_obj != lv_scr_act())
    {
      // 初始化表格格式
      lv_table_set_col_cnt(guider_ui.scrParams_tableDartParams, 2);
      lv_table_set_row_cnt(guider_ui.scrParams_tableDartParams, 0);
      lv_table_set_col_width(guider_ui.scrParams_tableDartParams, 0, 150);
      lv_table_set_col_width(guider_ui.scrParams_tableDartParams, 1, 150);
      
      lv_table_set_col_cnt(guider_ui.scrParams_tableDartStatus, 2);
      lv_table_set_row_cnt(guider_ui.scrParams_tableDartStatus, 0);
      lv_table_set_col_width(guider_ui.scrParams_tableDartStatus, 0, 150);
      lv_table_set_col_width(guider_ui.scrParams_tableDartStatus, 1, 150);
    }
    // 更新表格内容
    {
      // 序列化状态和参数
      json j_full = dart_launcher_status_;
      json j_params = j_full["params"];
      j_full.erase("params");
      j_full.erase("protocols");

      // 填充参数表格
      int rows_params = (int)j_params.size();
      lv_table_set_row_cnt(guider_ui.scrParams_tableDartParams, rows_params);
      int row = 0;
      for (auto& item : j_params.items()) {
        const std::string& key = item.key();
        std::string val = item.value().dump();
        lv_table_set_cell_value(guider_ui.scrParams_tableDartParams, row, 0, key.c_str());
        lv_table_set_cell_value(guider_ui.scrParams_tableDartParams, row, 1, val.c_str());
        row++;
      }

      // 填充状态表格
      int rows_status = (int)j_full.size();
      lv_table_set_row_cnt(guider_ui.scrParams_tableDartStatus, rows_status);
      row = 0;
      for (auto& item : j_full.items()) {
        const std::string& key = item.key();
        std::string val = item.value().dump();
        lv_table_set_cell_value(guider_ui.scrParams_tableDartStatus, row, 0, key.c_str());
        lv_table_set_cell_value(guider_ui.scrParams_tableDartStatus, row, 1, val.c_str());
        row++;
      }
    }
  }
  else if (lv_scr_act() == guider_ui.scrVision)
  {
    // 更新视觉界面，双缓冲显示图像
    LV_DRAW_BUF_DEFINE(draw_buf0, 426, 341, LV_COLOR_FORMAT_NATIVE);
    LV_DRAW_BUF_DEFINE(draw_buf1, 426, 341, LV_COLOR_FORMAT_NATIVE);
    static int buf_index = 0;
    buf_index %= 2;

    // 判断当前显示的是哪种画面
    const char *title = lv_label_get_text(guider_ui.scrVision_labelTitle);
    const sensor_msgs::msg::CompressedImage *msg = nullptr;
    if (strcmp(title, "制导相机画面") == 0)
    {
      msg = &greenlight_image_;
    }
    else if (strcmp(title, "参数相机画面") == 0)
    {
      msg = &qrcode_image_;
    }

    if (msg == nullptr || msg->data.empty())
      return;

    // 解码压缩图像为cv::Mat
    cv::Mat img_resized = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;

    cv::resize(img_resized, img_resized, cv::Size(426, 341), 0, 0, cv::INTER_LINEAR);

    if (img_resized.empty())
      return;

    // 转换为lvgl格式(BGR888->RGB565)
    lv_draw_buf_t *draw_buf = (buf_index == 0) ? &draw_buf0 : &draw_buf1;
    lv_image_header_t *header = &draw_buf->header;
    if (header->cf == LV_COLOR_FORMAT_RGB565)
    {
      uint32_t stride = header->stride;
      uint8_t *data = draw_buf->data;
      for (int row = 0; row < 341; ++row)
      {
        uint16_t *buf16 = (uint16_t *)(data + row * stride);
        for (int col = 0; col < 426; ++col)
        {
          auto px = img_resized.at<cv::Vec3b>(row, col);
          // BGR888 -> RGB565
          uint16_t r = px[2] >> 3;
          uint16_t g = px[1] >> 2;
          uint16_t b = px[0] >> 3;
          uint16_t rgb565 = (r << 11) | (g << 5) | b;
          buf16[col] = rgb565;
        }
      }
    }

    // 设置图像对象的src
    if (buf_index == 0)
      lv_canvas_set_draw_buf(guider_ui.scrVision_canvasVision, &draw_buf0);
    else
      lv_canvas_set_draw_buf(guider_ui.scrVision_canvasVision, &draw_buf1);
    buf_index++;
  }

  last_obj = lv_scr_act();
}

// 激活回调
CallbackReturn NodeDartApp::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  // 加载初始数据
  update_network_status();

  RCLCPP_INFO(get_logger(), "Enabling LVGL input...");
  // lv_linux_input_create();

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
    }); })
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
  greenlight_image_sub_.reset();
  green_light_sub_.reset();
  qrcode_image_sub_.reset();

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
    /*Linux display device init*/

    // 启动LVGL UI界面
    lv_init();
    lv_linux_disp_init();

    setup_ui(&guider_ui);
    custom_init(&guider_ui);
    // 设置UI Loader到33%
    lv_label_set_text(guider_ui.scrLoader_labelLoader, "0%");
    lv_arc_set_value(guider_ui.scrLoader_arcLoader, 0);
    lv_label_set_text(guider_ui.scrLoader_labelLoadStage, "等待配置...");

    auto now = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
      this->mutex_ui_.lock(); // 锁定互斥锁，防止在清理屏幕时更新GUI
      lv_timer_handler();
      // 每10毫秒执行一次主循环逻辑
      if (std::chrono::steady_clock::now() - now > 100ms)
      {
        now = std::chrono::steady_clock::now();
        screen_main_loop(); // 执行主循环逻辑
      }
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

// 更新网络状态
void NodeDartApp::update_network_status()
{
  static bool operating_ = false;
  if (operating_)
    return; // 防止重复操作
  // 自动检测wlan0和eth0，优先wlan0
  std::string ip = "未知";
  std::array<std::string, 2> interfaces = {"wlan0", "eth0"};
  bool found_ip = false;
  for (const auto &iface : interfaces)
  {
    std::string cmd = "ip addr show " + iface + " | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1";
    FILE *fp = popen(cmd.c_str(), "r");
    if (fp)
    {
      char buf[64] = {0};
      if (fgets(buf, sizeof(buf), fp))
      {
        ip = std::string(buf);
        ip.erase(ip.find_last_not_of(" \n\r\t") + 1); // 去除末尾换行
        found_ip = true;
      }
      pclose(fp);
    }
    if (found_ip)
      break;
  }
  this->ip_address_ = ip;

  // 判断网络可达性
  std::string ping_cmd = "ping -c 1 baidu.com > /dev/null 2>&1";
  int result = system(ping_cmd.c_str());
  this->is_online_ = (result == 0);

  // RCLCPP_INFO(get_logger(), "IP Addr: %s, IsOnline: %s", ip.c_str(), is_online_ ? "在线" : "离线");
}

// 更新Greenlight图像
void NodeDartApp::update_greenlight_image(sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  // 当且仅当当前屏幕是scrVision时，更新图像
  if (lv_scr_act() == guider_ui.scrVision)
  {
    greenlight_image_ = *msg;
  }
}

// 更新QRCode图像
void NodeDartApp::update_qrcode_image(sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  // 当且仅当当前屏幕是scrVision时，更新图像
  if (lv_scr_act() == guider_ui.scrVision)
  {
    qrcode_image_ = *msg;
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
