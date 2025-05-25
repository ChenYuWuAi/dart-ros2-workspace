#ifndef NODE_LIFECYCLE_DART_APP_HPP
#define NODE_LIFECYCLE_DART_APP_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <mutex>
#include <thread>
#include <string>
#include <filesystem>
#include <vector>
#include <deque>

// LVGL相关头文件
#include "lvgl/lvgl.h"
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <gui_guider.h>
#include <custom.h>

// ROS2消息类型
#include <dart_msgs/msg/dart_launcher_status.hpp>
#include <dart_msgs/msg/dart_launcher_params.hpp>
#include <dart_msgs/msg/green_light.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

// JSON
#include <nlohmann/json.hpp>

// SDL
#include <SDL2/SDL.h>

namespace fs = std::filesystem;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NodeDartApp : public rclcpp_lifecycle::LifecycleNode
{
public:
  NodeDartApp(rclcpp::NodeOptions options);
  virtual ~NodeDartApp();

  // 生命周期节点回调函数
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
  CallbackReturn on_error(const rclcpp_lifecycle::State &);
  bool reset_component_life_count(const std::string &component_name);

private:
  // 互斥锁
  std::mutex mutex_ui_;

  std::string ip_address_;
  bool is_online_ = false;

  // 线程
  std::shared_ptr<std::thread> ui_thread_;
  std::shared_ptr<std::thread> watch_file_thread_;

  // 计时器
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;

  // 订阅者
  rclcpp::Subscription<dart_msgs::msg::DartLauncherStatus>::SharedPtr dart_launcher_status_sub_;
  rclcpp::Subscription<dart_msgs::msg::DartLauncherParams>::SharedPtr dart_launcher_present_param_sub_;
  rclcpp::Subscription<dart_msgs::msg::GreenLight>::SharedPtr green_light_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr qrcode_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr greenlight_image_sub_;

  // 状态变量
  dart_msgs::msg::DartLauncherStatus dart_launcher_status_;
  bool mcu_online_ = false;
  std::chrono::time_point<std::chrono::steady_clock> last_status_time_;

  // 图像
  sensor_msgs::msg::CompressedImage qrcode_image_;
  sensor_msgs::msg::CompressedImage greenlight_image_;

  // 飞镖发射统计数据
  struct DartLaunchRecord
  {
    uint64_t time;     // 发射时间戳
    float speed;       // 发射速度
    uint32_t sequence; // 发射序号
  };
  std::string statistics_file_path_;                 // 统计数据文件路径
  std::deque<DartLaunchRecord> dart_launch_records_; // 最近的发射记录
  uint64_t last_recorded_launch_time_;               // 上次记录的发射时间
  uint32_t total_launch_count_;                      // 总发射次数

  // 元件寿命统计
  std::map<std::string, uint32_t> component_life_counts_; // 元件使用次数统计

  // 私有方法
  void timer_callback_ui();
  void update_greenlight_image(sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void update_qrcode_image(sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void update_network_status();
  void check_dart_launch();
  bool load_launch_statistics();
  bool save_launch_statistics();

  void screen_main_loop();

  // 友元函数模板，用于UI互斥锁
  template <typename Func>
  friend void with_ui_lock(std::shared_ptr<NodeDartApp> node, Func func);
};

// 互斥锁包装函数
template <typename Func>
void with_ui_lock(std::shared_ptr<NodeDartApp> node, Func func)
{
  std::lock_guard<std::mutex> lock(node->mutex_ui_);
  func(); // 调用传递的lambda表达式
}

// 全局UI实例
extern lv_ui guider_ui;
extern std::shared_ptr<NodeDartApp> g_node;

// 环境变量辅助函数
static const char *getenv_default(const char *name, const char *dflt)
{
  return getenv(name) ? getenv(name) : dflt;
}

#if LV_USE_LINUX_FBDEV
static void lv_linux_disp_init(void)
{
  const char *device = getenv_default("LV_LINUX_FBDEV_DEVICE", "/dev/fb0");
  lv_display_t *disp = lv_linux_fbdev_create();
  lv_linux_fbdev_set_file(disp, device);
}
#elif LV_USE_LINUX_DRM
static void lv_linux_disp_init(void)
{
  const char *device = getenv_default("LV_LINUX_DRM_CARD", "/dev/dri/by-path/platform-axi:gpu-card");
  lv_display_t *disp = lv_linux_drm_create();
  lv_linux_drm_set_file(disp, device, -1);
}
#elif LV_USE_SDL
static void lv_linux_disp_init(void)
{
  const int width = atoi(getenv("LV_SDL_VIDEO_WIDTH") ?: "800");
  const int height = atoi(getenv("LV_SDL_VIDEO_HEIGHT") ?: "480");
  lv_sdl_window_create(width, height);
  lv_sdl_mouse_create();

  SDL_ShowCursor(SDL_DISABLE);
}
#else
#error Unsupported configuration
#endif

#if LV_USE_EVDEV
static void lv_linux_input_create(void)
{
  const char *device = getenv_default("LV_LINUX_EVDEV_DEVICE", "/dev/input/touchscreen");
  lv_indev_t *indev = lv_evdev_create(LV_INDEV_TYPE_POINTER, device);
  lv_indev_enable(indev, true);
}
#endif

#endif // NODE_LIFECYCLE_DART_APP_HPP
