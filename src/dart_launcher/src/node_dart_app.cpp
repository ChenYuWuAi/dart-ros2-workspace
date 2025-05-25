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
NodeDartApp::NodeDartApp(rclcpp::NodeOptions options)
    : rclcpp_lifecycle::LifecycleNode("node_dart_app", options),
      last_recorded_launch_time_(0),
      total_launch_count_(0)
{
  RCLCPP_INFO(get_logger(), "Lifecycle node [%s] started.", get_name());

  // 声明并获取统计文件路径参数
  if (!has_parameter("statistics_file"))
  {
    this->declare_parameter("statistics_file", "/home/ubuntu/dart-ros2-workspace/src/dart_launcher/config/dart_launch_statistics.json");
    RCLCPP_WARN(get_logger(), "未设置统计文件路径，使用默认路径: /home/ubuntu/dart-ros2-workspace/src/dart_launcher/config/dart_launch_statistics.json");
  }
  statistics_file_path_ = this->get_parameter("statistics_file").as_string();
  RCLCPP_INFO(get_logger(), "飞镖发射统计数据将保存至: %s", statistics_file_path_.c_str());

  // 加载已有统计数据
  if (!load_launch_statistics())
  {
    RCLCPP_WARN(get_logger(), "无法加载统计数据，将创建新的统计记录");
  }

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

        // 如果发射时间变化，说明有新的发射
        if (msg->last_launch_time != dart_launcher_status_.last_launch_time &&
            msg->last_launch_time > 0)
        {
          check_dart_launch();
        }

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
      lv_obj_add_flag(guider_ui.scrHome_imgIconWIFI, LV_OBJ_FLAG_HIDDEN);     // 隐藏WiFi图标
      lv_obj_clear_flag(guider_ui.scrHome_imgIconNoWIFI, LV_OBJ_FLAG_HIDDEN); // 显示无网络图标
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
        lv_label_set_text(guider_ui.scrHome_labelMCUMode, "比赛Enter");
        break;
      case 104:
        lv_label_set_text(guider_ui.scrHome_labelMCUMode, "比赛Wait");
        break;
      case 105:
        lv_label_set_text(guider_ui.scrHome_labelMCUMode, "比赛Launch");
        break;
      case 106:
        lv_label_set_text(guider_ui.scrHome_labelMCUMode, "比赛Reload");
        break;
      case 107:
        lv_label_set_text(guider_ui.scrHome_labelMCUMode, "比赛End");
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
      json j_params;
      if (param_edit_is_param == 0)
        j_params = j_full["params"];
      else
        j_params = j_full["protocols"];

      j_full.erase("params");
      j_full.erase("protocols");
      j_full.erase("header");

      // 填充参数表格
      int rows_params = (int)j_params.size();
      lv_table_set_row_cnt(guider_ui.scrParams_tableDartParams, rows_params);
      int row = 0;
      for (auto &item : j_params.items())
      {
        const std::string &key = item.key();
        std::string val = item.value().dump();
        lv_table_set_cell_value(guider_ui.scrParams_tableDartParams, row, 0, key.c_str());
        lv_table_set_cell_value(guider_ui.scrParams_tableDartParams, row, 1, val.c_str());
        row++;
      }

      // 填充状态表格
      int rows_status = (int)j_full.size();
      lv_table_set_row_cnt(guider_ui.scrParams_tableDartStatus, rows_status);
      row = 0;
      for (auto &item : j_full.items())
      {
        const std::string &key = item.key();
        std::string val = item.value().dump();
        lv_table_set_cell_value(guider_ui.scrParams_tableDartStatus, row, 0, key.c_str());
        lv_table_set_cell_value(guider_ui.scrParams_tableDartStatus, row, 1, val.c_str());
        row++;
      }
    }
  }
  else if (lv_scr_act() == guider_ui.scrQRCode)
  {
    // 限制调用频率
    static auto last_update_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time).count() < 500 && last_obj == lv_scr_act())
      return; // 每500毫秒更新一次
    // 序列化状态和参数
    json j_full = dart_launcher_status_;
    json j_params;
    // 根据scrQRCode_tb

    if (param_edit_is_param == 0)
    {
      j_params["command_type"] = "DartParams";
      j_params["data"] = j_full["params"];
    }
    else
    {
      j_params["command_type"] = "DartProtocols";
      j_params["data"] = j_full["protocols"];
    }

    // j_params序列化后填入
    lv_qrcode_update(guider_ui.scrQRCode_qrcodeExport, j_params.dump().c_str(), j_params.dump().size());
    last_update_time = now;
  }

  else if (lv_scr_act() == guider_ui.scrStatistic)
  {
    // 更新统计屏幕
    if (last_obj != lv_scr_act())
    {
      // 初始化发射记录表格
      lv_table_set_row_cnt(guider_ui.scrStatistic_tableDartLaunches, dart_launch_records_.size() + 1);
      lv_table_set_cell_value(guider_ui.scrStatistic_tableDartLaunches, 0, 0, "序号");
      lv_table_set_cell_value(guider_ui.scrStatistic_tableDartLaunches, 0, 1, "时间");
      lv_table_set_cell_value(guider_ui.scrStatistic_tableDartLaunches, 0, 2, "速度");

      // 初始化元件寿命表格
      lv_table_set_row_cnt(guider_ui.scrStatistic_tableLifeSpan, 7); // 表头 + 6个元件
      lv_table_set_cell_value(guider_ui.scrStatistic_tableLifeSpan, 0, 0, "元件");
      lv_table_set_cell_value(guider_ui.scrStatistic_tableLifeSpan, 0, 1, "次数");

      // 初始化下拉菜单
      lv_dropdown_clear_options(guider_ui.scrStatistic_ddlistLifeSpanSelect);
      lv_dropdown_add_option(guider_ui.scrStatistic_ddlistLifeSpanSelect, "总次数", LV_DROPDOWN_POS_LAST);
      lv_dropdown_add_option(guider_ui.scrStatistic_ddlistLifeSpanSelect, "滑台", LV_DROPDOWN_POS_LAST);
      lv_dropdown_add_option(guider_ui.scrStatistic_ddlistLifeSpanSelect, "扳机", LV_DROPDOWN_POS_LAST);
      lv_dropdown_add_option(guider_ui.scrStatistic_ddlistLifeSpanSelect, "缓冲打印件", LV_DROPDOWN_POS_LAST);
      lv_dropdown_add_option(guider_ui.scrStatistic_ddlistLifeSpanSelect, "下滑块", LV_DROPDOWN_POS_LAST);
      lv_dropdown_add_option(guider_ui.scrStatistic_ddlistLifeSpanSelect, "皮筋", LV_DROPDOWN_POS_LAST);
    }

    // 更新发射记录表格数据
    if (dart_launch_records_.size() > 0)
    {
      int row = 1;
      for (auto it = dart_launch_records_.rbegin(); it != dart_launch_records_.rend(); ++it)
      {
        // 序号列
        std::string seq_text = std::to_string(it->sequence);
        lv_table_set_cell_value(guider_ui.scrStatistic_tableDartLaunches, row, 0, seq_text.c_str());

        // 时间列
        std::time_t time_val = static_cast<std::time_t>(it->time / 1000); // 假设时间戳是毫秒
        std::tm tm_info;
        localtime_r(&time_val, &tm_info);
        char time_str[64];
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_info);
        lv_table_set_cell_value(guider_ui.scrStatistic_tableDartLaunches, row, 1, time_str);

        // 速度列
        std::string speed_text = std::to_string(it->speed);
        // 保留两位小数
        size_t pos = speed_text.find('.');
        if (pos != std::string::npos && pos + 3 < speed_text.size())
        {
          speed_text = speed_text.substr(0, pos + 3);
        }
        lv_table_set_cell_value(guider_ui.scrStatistic_tableDartLaunches, row, 2, speed_text.c_str());

        row++;
      }
    }

    // 更新元件寿命表格
    std::vector<std::pair<std::string, std::string>> components = {
        {"总发射次数", "total"},
        {"滑台", "slider"},
        {"扳机", "trigger"},
        {"缓冲打印件", "buffer"},
        {"下滑块", "bottom"},
        {"皮筋", "rubber"}};

    for (size_t i = 0; i < components.size(); i++)
    {
      lv_table_set_cell_value(guider_ui.scrStatistic_tableLifeSpan, i + 1, 0, components[i].first.c_str());
      std::string count_str = std::to_string(component_life_counts_[components[i].second]);
      lv_table_set_cell_value(guider_ui.scrStatistic_tableLifeSpan, i + 1, 1, count_str.c_str());
    }
  }
  else if (lv_scr_act() == guider_ui.scrVision)
  {
    // 确保 canvas 已初始化
    if (!guider_ui.scrVision_canvasVision)
      return;

    try
    {
      // 更新视觉界面，双缓冲显示图像
      LV_DRAW_BUF_DEFINE(draw_buf0, 426, 341, LV_COLOR_FORMAT_NATIVE);
      LV_DRAW_BUF_DEFINE(draw_buf1, 426, 341, LV_COLOR_FORMAT_NATIVE);
      static int buf_index = 0;
      buf_index %= 2;

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
      if (!msg || msg->data.empty())
        return;

      cv::Mat img = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
      if (img.empty())
        return;
      cv::resize(img, img, cv::Size(426, 341), 0, 0, cv::INTER_LINEAR);

      lv_draw_buf_t *draw_buf = (buf_index == 0) ? &draw_buf0 : &draw_buf1;
      lv_image_header_t *header = &draw_buf->header;
      if (header->cf == LV_COLOR_FORMAT_RGB565 && draw_buf->data)
      {
        uint32_t stride = header->stride;
        uint8_t *data = draw_buf->data;
        for (int row = 0; row < 341; ++row)
        {
          uint16_t *buf16 = reinterpret_cast<uint16_t *>(data + row * stride);
          for (int col = 0; col < 426; ++col)
          {
            auto px = img.at<cv::Vec3b>(row, col);
            uint16_t r = px[2] >> 3;
            uint16_t g = px[1] >> 2;
            uint16_t b = px[0] >> 3;
            buf16[col] = (r << 11) | (g << 5) | b;
          }
        }
      }

      // 检查缓冲有效性
      if (!draw_buf->data || draw_buf->header.cf != LV_COLOR_FORMAT_RGB565)
      {
        RCLCPP_ERROR(get_logger(), "画布缓冲无效或格式不匹配");
        return;
      }
      // 切换缓冲
      if (buf_index == 0)
        lv_canvas_set_draw_buf(guider_ui.scrVision_canvasVision, &draw_buf0);
      else
        lv_canvas_set_draw_buf(guider_ui.scrVision_canvasVision, &draw_buf1);
      buf_index++;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "scrVision 更新异常: %s", e.what());
    }
  }
  else if (lv_scr_act() == guider_ui.scrSetup)
  {
    // 更新系统信息，格式
    // ====网络信息====
    // IPv4:
    // IPv6:
    // SSID:
    // ====负载信息====
    // Temp:
    // CPU:
    // Mem:
    // 控制更新频率
    static auto last_update_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time).count() < 1000 && last_obj == lv_scr_act())
      return; // 每1000毫秒更新一次
    std::string sysinfo;

    // ====网络信息====
    sysinfo += "====网络信息====\n";
    // IPv4
    sysinfo += "IPv4: " + ip_address_ + "\n";

    // IPv6
    std::string ipv6 = "N/A";
    std::array<std::string, 2> interfaces = {"wlan0", "eth0"};
    for (const auto &iface : interfaces)
    {
      std::string cmd = "ip -6 addr show " + iface + " | grep 'inet6 ' | awk '{print $2}' | cut -d'/' -f1 | head -n 1";
      FILE *fp = popen(cmd.c_str(), "r");
      if (fp)
      {
        char buf[128] = {0};
        if (fgets(buf, sizeof(buf), fp))
        {
          ipv6 = std::string(buf);
          ipv6.erase(ipv6.find_last_not_of(" \n\r\t") + 1);
          if (!ipv6.empty())
            break;
        }
        pclose(fp);
      }
    }
    sysinfo += "IPv6: " + ipv6 + "\n";

    // SSID
    std::string ssid = "N/A";
    std::string cmd_ssid = "iwgetid -r";
    FILE *fp_ssid = popen(cmd_ssid.c_str(), "r");
    if (fp_ssid)
    {
      char buf[128] = {0};
      if (fgets(buf, sizeof(buf), fp_ssid))
      {
        ssid = std::string(buf);
        ssid.erase(ssid.find_last_not_of(" \n\r\t") + 1);
      }
      pclose(fp_ssid);
    }
    sysinfo += "SSID: " + ssid + "\n";

    // ====负载信息====
    sysinfo += "====负载信息====\n";
    // Temp
    std::string temp = "N/A";
    FILE *fp_temp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (fp_temp)
    {
      int t = 0;
      if (fscanf(fp_temp, "%d", &t) == 1)
      {
        char temp_buf[32];
        snprintf(temp_buf, sizeof(temp_buf), "%.1f'C", t / 1000.0);
        temp = temp_buf;
      }
      fclose(fp_temp);
    }
    sysinfo += "Temp: " + temp + "\n";

    // CPU
    double cpu_usage = 0.0;
    static long last_total = 0, last_idle = 0;
    FILE *fp_cpu = fopen("/proc/stat", "r");
    if (fp_cpu)
    {
      char line[256];
      if (fgets(line, sizeof(line), fp_cpu))
      {
        long user, nice, system, idle, iowait, irq, softirq, steal;
        sscanf(line, "cpu  %ld %ld %ld %ld %ld %ld %ld %ld",
               &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);
        long total = user + nice + system + idle + iowait + irq + softirq + steal;
        long diff_total = total - last_total;
        long diff_idle = idle - last_idle;
        if (last_total != 0 && diff_total > 0)
        {
          cpu_usage = 100.0 * (diff_total - diff_idle) / diff_total;
        }
        last_total = total;
        last_idle = idle;
      }
      fclose(fp_cpu);
    }
    char cpu_buf[32];
    snprintf(cpu_buf, sizeof(cpu_buf), "%.1f%%", cpu_usage);
    sysinfo += "CPU: " + std::string(cpu_buf) + "\n";

    // Mem
    long mem_total = 0, mem_free = 0, mem_available = 0;
    FILE *fp_mem = fopen("/proc/meminfo", "r");
    if (fp_mem)
    {
      char key[64];
      long value;
      while (fscanf(fp_mem, "%63s %ld kB\n", key, &value) == 2)
      {
        if (strcmp(key, "MemTotal:") == 0)
          mem_total = value;
        else if (strcmp(key, "MemAvailable:") == 0)
          mem_available = value;
      }
      fclose(fp_mem);
    }
    char mem_buf[64];
    if (mem_total > 0)
    {
      snprintf(mem_buf, sizeof(mem_buf), "%.1f%%", 100.0 * (mem_total - mem_available) / mem_total);
      sysinfo += "Mem: " + std::string(mem_buf) + "\n";
    }
    else
    {
      sysinfo += "Mem: N/A\n";
    }

    // 更新到UI
    lv_label_set_text(guider_ui.scrSetup_labelSystemStatus, sysinfo.c_str());
    last_update_time = now;
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

  // 切换 UI 到停用提示
  with_ui_lock(g_node, [&]()
               {
    lv_screen_load(guider_ui.scrLoader);
    lv_label_set_text(guider_ui.scrLoader_labelLoadStage, "节点停用...");
    lv_label_set_text(guider_ui.scrLoader_labelPrompt, "请等待");
    lv_label_set_text(guider_ui.scrLoader_labelLoader, "0%");
    lv_arc_set_value(guider_ui.scrLoader_arcLoader, 0); });

  return CallbackReturn::SUCCESS;
}

// 清理回调
CallbackReturn NodeDartApp::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "正在清理...");

  // 切换 UI 到清理提示
  with_ui_lock(g_node, [&]()
               {
    lv_label_set_text(guider_ui.scrLoader_labelLoadStage, "清理资源...");
    lv_label_set_text(guider_ui.scrLoader_labelLoader, "0%");
    lv_label_set_text(guider_ui.scrLoader_labelPrompt, "请等待");
    lv_arc_set_value(guider_ui.scrLoader_arcLoader, 0);
    lv_screen_load(guider_ui.scrLoader); });

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

  // 切换 UI 到关闭提示
  with_ui_lock(g_node, [&]()
               {
    lv_label_set_text(guider_ui.scrLoader_labelLoadStage, "节点清理...");
    lv_label_set_text(guider_ui.scrLoader_labelLoader, "0%");
    lv_label_set_text(guider_ui.scrLoader_labelPrompt, "请等待");
    lv_arc_set_value(guider_ui.scrLoader_arcLoader, 0);
    lv_screen_load(guider_ui.scrLoader); });

  // 等待 UI 线程完成
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
  std::string ip = "N/A";
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

// 检查飞镖发射并更新统计数据
void NodeDartApp::check_dart_launch()
{
  // 确保发射时间已经更新
  if (dart_launcher_status_.last_launch_time <= last_recorded_launch_time_)
  {
    return;
  }

  // 更新记录的发射时间
  RCLCPP_INFO(get_logger(), "检测到新的飞镖发射，时间: %lu, 速度: %.2f",
              dart_launcher_status_.last_launch_time,
              dart_launcher_status_.last_launch_speed);

  // 创建新的发射记录
  DartLaunchRecord record;
  record.time = dart_launcher_status_.last_launch_time;
  record.speed = dart_launcher_status_.last_launch_speed;
  record.sequence = ++total_launch_count_;

  // 添加到发射记录队列
  dart_launch_records_.push_back(record);

  // 限制记录队列大小，保留最新的20条记录
  while (dart_launch_records_.size() > 20)
  {
    dart_launch_records_.pop_front();
  }

  // 更新各个元件的使用次数
  component_life_counts_["total"]++;   // 总发射次数
  component_life_counts_["slider"]++;  // 滑台
  component_life_counts_["trigger"]++; // 扳机
  component_life_counts_["buffer"]++;  // 缓冲打印件
  component_life_counts_["bottom"]++;  // 下滑块
  component_life_counts_["rubber"]++;  // 皮筋

  // 保存到文件
  last_recorded_launch_time_ = dart_launcher_status_.last_launch_time;
  save_launch_statistics();
}

// 加载飞镖发射统计数据
bool NodeDartApp::load_launch_statistics()
{
  try
  {
    // 如果文件不存在，返回false
    if (!fs::exists(statistics_file_path_))
    {
      RCLCPP_INFO(get_logger(), "统计文件不存在: %s, 将创建新文件", statistics_file_path_.c_str());

      // 初始化元件寿命计数器
      component_life_counts_["total"] = 0;   // 总发射次数
      component_life_counts_["slider"] = 0;  // 滑台
      component_life_counts_["trigger"] = 0; // 扳机
      component_life_counts_["buffer"] = 0;  // 缓冲打印件
      component_life_counts_["bottom"] = 0;  // 下滑块
      component_life_counts_["rubber"] = 0;  // 皮筋
      return false;
    }

    // 打开并读取JSON文件
    std::ifstream file(statistics_file_path_);
    if (!file.is_open())
    {
      RCLCPP_ERROR(get_logger(), "无法打开统计文件: %s", statistics_file_path_.c_str());
      return false;
    }

    // 解析JSON
    nlohmann::json json_data;
    file >> json_data;
    file.close();

    // 提取数据
    if (json_data.contains("total_launch_count"))
    {
      total_launch_count_ = json_data["total_launch_count"];
    }

    if (json_data.contains("launches") && json_data["launches"].is_array())
    {
      dart_launch_records_.clear();
      for (const auto &record : json_data["launches"])
      {
        DartLaunchRecord dart_record;
        dart_record.time = record["time"];
        dart_record.speed = record["speed"];
        dart_record.sequence = record["sequence"];
        dart_launch_records_.push_back(dart_record);
      }
    }

    // 加载元件寿命数据
    if (json_data.contains("component_life"))
    {
      const auto &life_data = json_data["component_life"];

      // 初始化默认值
      component_life_counts_["total"] = life_data.value("total", 0);     // 总发射次数
      component_life_counts_["slider"] = life_data.value("slider", 0);   // 滑台
      component_life_counts_["trigger"] = life_data.value("trigger", 0); // 扳机
      component_life_counts_["buffer"] = life_data.value("buffer", 0);   // 缓冲打印件
      component_life_counts_["bottom"] = life_data.value("bottom", 0);   // 下滑块
      component_life_counts_["rubber"] = life_data.value("rubber", 0);   // 皮筋
    }
    else
    {
      // 如果没有元件寿命数据，则初始化为0
      component_life_counts_["total"] = 0;   // 总发射次数
      component_life_counts_["slider"] = 0;  // 滑台
      component_life_counts_["trigger"] = 0; // 扳机
      component_life_counts_["buffer"] = 0;  // 缓冲打印件
      component_life_counts_["bottom"] = 0;  // 下滑块
      component_life_counts_["rubber"] = 0;  // 皮筋
    }

    // 设置上次记录时间为最近一条记录的时间
    if (!dart_launch_records_.empty())
    {
      last_recorded_launch_time_ = dart_launch_records_.back().time;
    }

    RCLCPP_INFO(get_logger(), "已加载统计数据: 总计 %u 次发射, %zu 条记录",
                total_launch_count_, dart_launch_records_.size());
    return true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "加载统计数据时出错: %s", e.what());
    return false;
  }
}

// 保存飞镖发射统计数据
bool NodeDartApp::save_launch_statistics()
{
  try
  {
    // 创建JSON数据
    nlohmann::json json_data;
    json_data["total_launch_count"] = total_launch_count_;

    // 添加发射记录
    nlohmann::json launches_array = nlohmann::json::array();
    for (const auto &record : dart_launch_records_)
    {
      nlohmann::json record_json;
      record_json["time"] = record.time;
      record_json["speed"] = record.speed;
      record_json["sequence"] = record.sequence;
      launches_array.push_back(record_json);
    }
    json_data["launches"] = launches_array;

    // 添加元件寿命数据
    nlohmann::json component_life;
    for (const auto &[key, value] : component_life_counts_)
    {
      component_life[key] = value;
    }
    json_data["component_life"] = component_life;

    // 写入文件
    std::ofstream file(statistics_file_path_);
    if (!file.is_open())
    {
      RCLCPP_ERROR(get_logger(), "无法打开统计文件进行写入: %s", statistics_file_path_.c_str());
      return false;
    }

    file << json_data.dump(2); // 使用2个空格缩进使JSON更易读
    file.close();

    RCLCPP_INFO(get_logger(), "统计数据已保存至: %s", statistics_file_path_.c_str());
    return true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "保存统计数据时出错: %s", e.what());
    return false;
  }
}

// 重置元件寿命计数器
bool NodeDartApp::reset_component_life_count(const std::string &component_name)
{
  // 检查是否是有效的元件名称
  if (component_life_counts_.find(component_name) != component_life_counts_.end())
  {
    RCLCPP_INFO(get_logger(), "重置元件 %s 的寿命计数器", component_name.c_str());

    // 重置计数器
    component_life_counts_[component_name] = 0;

    // 保存到文件
    return save_launch_statistics();
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "未知的元件名称: %s", component_name.c_str());
    return false;
  }
}

// UI调用的桥接函数，用于重置元件寿命
extern "C" bool reset_component_life_count_from_ui(const char *component_name)
{
  if (g_node)
  {
    return g_node->reset_component_life_count(component_name);
  }
  return false;
}

// 联网
extern "C" void connect_to_wifi(const char *ssid)
{
  static bool operating = false;

  // 检查合法性
  if (!ssid || std::string(ssid).empty())
  {
    RCLCPP_ERROR(g_node->get_logger(), "SSID不能为空");
    return;
  }

  if (operating)
    return;
  operating = true;

  RCLCPP_INFO(g_node->get_logger(), "正在尝试连接WiFi: %s", ssid);

  std::string ssid_str = ssid;
  std::thread([ssid_str]() {
    std::string cmd = "nmcli con up \"" + ssid_str + "\"";
    int ret = system(cmd.c_str());
    if (ret == 0) {
      RCLCPP_INFO(g_node->get_logger(), "已尝试连接WiFi: %s", ssid_str.c_str());
    } else {
      RCLCPP_ERROR(g_node->get_logger(), "连接WiFi失败: %s", ssid_str.c_str());
    }
    operating = false;
  }).detach();
}

// 主函数
int main(int argc, char **argv)
{
  // 初始化ROS
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);
  // 创建生命周期节点
  g_node = std::make_shared<NodeDartApp>(options);
  RCLCPP_INFO(g_node->get_logger(), "Node started. Spinning...");
  rclcpp::spin(g_node->get_node_base_interface());

  // 清理资源
  rclcpp::shutdown();

  return 0;
}
