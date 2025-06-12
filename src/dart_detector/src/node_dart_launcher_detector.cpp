#include <node_launcher_detector.hpp>
#include <fstream>
#include <sys/stat.h>
#include <cerrno>
#include <cstring>
#include <filesystem>

using namespace std::chrono_literals;
using namespace CameraHAL;

void NodeDartLauncherDetector::camera_thread(std::shared_ptr<CameraDriver> camera, const std::string &camera_name, bool is_qr_detection)
{
    RCLCPP_INFO(this->get_logger(), "Starting thread for %s camera...", camera_name.c_str());

    // 重置相机失败计数器
    if (camera_name == "lccv")
    {
        lccv_failure_count_ = 0;
        lccv_working_ = false;
    }
    else if (camera_name == "dh")
    {
        dh_failure_count_ = 0;
        dh_working_ = false;
    }

    // 获取相机看门狗阈值（连续失败多少次触发重启）
    int watchdog_threshold = this->has_parameter("camera_watchdog.failure_threshold") ? this->get_parameter("camera_watchdog.failure_threshold").as_int() : 30;

    int width = this->has_parameter(camera_name + ".image_width") ? this->get_parameter(camera_name + ".image_width").as_int() : 1280;
    int height = this->has_parameter(camera_name + ".image_height") ? this->get_parameter(camera_name + ".image_height").as_int() : 1024;
    int width_resized = this->has_parameter(camera_name + ".image_resized_width") ? this->get_parameter(camera_name + ".image_resized_width").as_int() : 640;
    int height_resized = this->has_parameter(camera_name + ".image_resized_height") ? this->get_parameter(camera_name + ".image_resized_height").as_int() : 512;
    bool resize = this->has_parameter(camera_name + ".image_resize_enable") ? this->get_parameter(camera_name + ".image_resize_enable").as_bool() : false;

    bool save_video = this->has_parameter(camera_name + ".save_video.enable") ? this->get_parameter(camera_name + ".save_video.enable").as_bool() : false;

    cv::VideoWriter video_writer;
    double video_write_fps = 15.0;
    if (save_video)
    {
        std::string video_path = this->has_parameter(camera_name + ".save_video.path") ? this->get_parameter(camera_name + ".save_video.path").as_string() : "./" + camera_name + "_video/";

        // 创建目录
        std::filesystem::create_directories(video_path);

        // 按日期和相机名生成文件名
        std::string timestamp = std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        std::string video_file = video_path + "/" + camera_name + "_" + timestamp + ".avi";

        // 使用硬件加速编码器
        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        video_write_fps = this->has_parameter(camera_name + ".save_video.fps") ? this->get_parameter(camera_name + ".save_video.fps").as_double() : 15.0;

        // 创建视频写入器
        video_writer.open(video_file, fourcc, video_write_fps, cv::Size(resize ? width_resized : width, resize ? height_resized : height));

        if (!video_writer.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file for writing: %s", video_file.c_str());
            save_video = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Saving video to: %s at %f FPS", video_file.c_str(), video_write_fps);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Camera %s runtime parameters: %dx%d, resized: %dx%d, resize_enable: %s",
                camera_name.c_str(), width, height, width_resized, height_resized, resize ? "true" : "false");

    cv::Mat image(height, width, CV_8UC3);
    cv::Mat image_resized(height_resized, width_resized, CV_8UC3);
    if (is_qr_detection)
    {
        // Activate publishers
        qr_image_publisher_->on_activate();
        qr_detect_publisher_->on_activate();

        std::string last_detected_qr_code_str;
        int reset_last_detected_qr_code_counter = 0;

        double target_fps = this->has_parameter(camera_name + ".camera_params.FrameRate")
                                ? static_cast<double>(this->get_parameter(camera_name + ".camera_params.FrameRate").as_int())
                                : 30.0;
        double target_sleep_time_ms = 1000.0 / target_fps;
        RCLCPP_INFO(this->get_logger(), "Target FPS: %.2f, Target sleep time: %.2f ms", target_fps, target_sleep_time_ms);

        while (running_ && rclcpp::ok())
        {
            if (!camera->read(image))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to read image from %s camera", camera_name.c_str());

                // 增加失败计数
                if (camera_name == "lccv")
                {
                    lccv_failure_count_++;
                    lccv_working_ = false;
                    RCLCPP_DEBUG(this->get_logger(), "LCCV camera failure count: %d", lccv_failure_count_.load());
                    if (lccv_failure_count_ >= watchdog_threshold && camera_watchdog_active_)
                    {
                        RCLCPP_ERROR(this->get_logger(), "LCCV camera failure threshold reached, triggering node restart");
                        try_restart_node();
                        return; // 退出线程
                    }
                }
                else if (camera_name == "dh")
                {
                    dh_failure_count_++;
                    dh_working_ = false;
                    RCLCPP_DEBUG(this->get_logger(), "DH camera failure count: %d", dh_failure_count_.load());
                    if (dh_failure_count_ >= watchdog_threshold && camera_watchdog_active_)
                    {
                        RCLCPP_ERROR(this->get_logger(), "DH camera failure threshold reached, triggering node restart");
                        try_restart_node();
                        return; // 退出线程
                    }
                }

                std::this_thread::sleep_for(16ms);
                continue;
            }

            // 重置失败计数
            if (camera_name == "lccv")
            {
                lccv_failure_count_ = 0;
                lccv_working_ = true;
            }
            else if (camera_name == "dh")
            {
                dh_failure_count_ = 0;
                dh_working_ = true;
            }

            RCLCPP_DEBUG(this->get_logger(), "Image read successfully from %s camera", camera_name.c_str());

            static double video_accum_time = 0.0;
            static std::chrono::steady_clock::time_point last_video_frame_time = std::chrono::steady_clock::now();
            if (save_video && video_writer.isOpened())
            {
                double video_frame_interval = 1000.0 / video_write_fps;
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_video_frame_time).count();
                video_accum_time += elapsed;
                if (video_accum_time >= video_frame_interval)
                {
                    video_writer.write(resize ? (cv::resize(image, image_resized, cv::Size(width_resized, height_resized)), image_resized) : image);
                    video_accum_time = 0.0;
                    last_video_frame_time = now;
                }
                else
                {
                    last_video_frame_time = now;
                }
            }

            auto start_time = std::chrono::steady_clock::now();
            auto qr_codes = qr_detector_.detect(image);
            if (!qr_codes.empty() && qr_codes[0] != last_detected_qr_code_str)
            {
                RCLCPP_INFO(this->get_logger(), "Detected QR codes from %s: %s", camera_name.c_str(), qr_codes[0].c_str());
                if (this->has_parameter("detect.qr_detect.save_image.enable") && this->get_parameter("detect.qr_detect.save_image.enable").as_bool())
                {
                    RCLCPP_INFO(this->get_logger(), "Saving QR code image to disk...");
                    if (this->has_parameter("detect.qr_detect.save_image.path"))
                        cv::imwrite(this->get_parameter("detect.qr_detect.save_image.path").as_string(), image);
                    else
                        cv::imwrite("./qr_image.jpg", image);
                }
                std_msgs::msg::String qr_msg;
                qr_msg.data = qr_codes[0];
                qr_detect_publisher_->publish(qr_msg);
                last_detected_qr_code_str = qr_codes[0];
            }
            else
            {
                if (!qr_codes.empty())
                {
                    RCLCPP_DEBUG(this->get_logger(), "QR code detected but not new: %s", qr_codes[0].c_str());
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "No QR code detected in current frame of %s camera", camera_name.c_str());
                    if (last_detected_qr_code_str != "")
                    {
                        reset_last_detected_qr_code_counter++;
                        if (reset_last_detected_qr_code_counter > 30)
                        {
                            last_detected_qr_code_str = "";
                            reset_last_detected_qr_code_counter = 0;
                        }
                    }
                }
            }

            std_msgs::msg::Header header;
            header.stamp = this->now();
            auto image_msg = cv_bridge::CvImage(header, "bgr8", image).toCompressedImageMsg();

            qr_image_publisher_->publish(*image_msg);
            auto end_time = std::chrono::steady_clock::now();
            auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            auto sleep_time = target_sleep_time_ms - processing_time;
            if (sleep_time > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time)));
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                     10000, "QR Detection processing time exceeded target frame time: %ld ms", processing_time);
            }
        }
        qr_image_publisher_->on_deactivate();
        qr_detect_publisher_->on_deactivate();
    }
    else
    {

        double alpha = this->has_parameter("greenlight.lowpass_filter.alpha")
                           ? this->get_parameter("greenlight.lowpass_filter.alpha").as_double()
                           : 0.5;

        RCLCPP_INFO(this->get_logger(), "Runtime using lowpass filter with alpha: %.2f", alpha);

        bool previous_detection = false;
        greenlight_publisher_->on_activate();
        greenlight_image_publisher_->on_activate();
        double target_fps = this->has_parameter(camera_name + ".camera_params.FPS")
                                ? static_cast<double>(this->get_parameter(camera_name + ".camera_params.FPS").as_int())
                                : 30.0;
        double target_sleep_time_ms = 1000.0 / target_fps;

        RCLCPP_INFO(this->get_logger(), "Target FPS: %.2f, Target sleep time: %.2f ms", target_fps, target_sleep_time_ms);

        while (running_ && rclcpp::ok())
        {

            try
            {
                if (!camera->read(image))
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to read image from %s camera", camera_name.c_str());

                    // 增加失败计数
                    if (camera_name == "lccv")
                    {
                        lccv_failure_count_++;
                        lccv_working_ = false;
                        RCLCPP_DEBUG(this->get_logger(), "LCCV camera failure count: %d", lccv_failure_count_.load());
                        if (lccv_failure_count_ >= watchdog_threshold && camera_watchdog_active_)
                        {
                            RCLCPP_ERROR(this->get_logger(), "LCCV camera failure threshold reached, triggering node restart");
                            try_restart_node();
                            return; // 退出线程
                        }
                    }
                    else if (camera_name == "dh")
                    {
                        dh_failure_count_++;
                        dh_working_ = false;
                        RCLCPP_DEBUG(this->get_logger(), "DH camera failure count: %d", dh_failure_count_.load());
                        if (dh_failure_count_ >= watchdog_threshold && camera_watchdog_active_)
                        {
                            RCLCPP_ERROR(this->get_logger(), "DH camera failure threshold reached, triggering node restart");
                            try_restart_node();
                            return; // 退出线程
                        }
                    }

                    std::this_thread::sleep_for(16ms);
                    continue;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception while reading image from %s camera: %s", camera_name.c_str(), e.what());

                // 增加失败计数
                if (camera_name == "lccv")
                {
                    lccv_failure_count_++;
                    lccv_working_ = false;
                    if (lccv_failure_count_ >= watchdog_threshold && camera_watchdog_active_)
                    {
                        RCLCPP_ERROR(this->get_logger(), "LCCV camera failure threshold reached (exception), triggering node restart");
                        try_restart_node();
                        return; // 退出线程
                    }
                }
                else if (camera_name == "dh")
                {
                    dh_failure_count_++;
                    dh_working_ = false;
                    if (dh_failure_count_ >= watchdog_threshold && camera_watchdog_active_)
                    {
                        RCLCPP_ERROR(this->get_logger(), "DH camera failure threshold reached (exception), triggering node restart");
                        try_restart_node();
                        return; // 退出线程
                    }
                }

                std::this_thread::sleep_for(16ms);
                continue;
            }

            // 重置失败计数
            if (camera_name == "lccv")
            {
                lccv_failure_count_ = 0;
                lccv_working_ = true;
            }
            else if (camera_name == "dh")
            {
                dh_failure_count_ = 0;
                dh_working_ = true;
            }

            RCLCPP_DEBUG(this->get_logger(), "Image read successfully from %s camera", camera_name.c_str());
            auto start_time = std::chrono::steady_clock::now();

            bool is_detected = false;
            double x = 0, y = 0;
            if (image.empty())
                continue;

            if (resize)
            {
                cv::resize(image, image_resized, cv::Size(width_resized, height_resized));
                if (image_resized.empty())
                {
                    RCLCPP_WARN(this->get_logger(), "Resized image is empty");
                    continue;
                }
                perform_greenlight_detection(image_resized, is_detected, x, y);
            }
            else
                perform_greenlight_detection(image, is_detected, x, y);

            if (is_detected)
            {
                RCLCPP_DEBUG(this->get_logger(), "%s camera detected green light at (%.2f, %.2f)", camera_name.c_str(), x, y);
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "No green light detected in current frame of %s camera", camera_name.c_str());
            }

            static double video_accum_time = 0.0;
            static std::chrono::steady_clock::time_point last_video_frame_time = std::chrono::steady_clock::now();
            if (save_video && video_writer.isOpened())
            {
                double video_frame_interval = 1000.0 / video_write_fps;
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_video_frame_time).count();
                video_accum_time += elapsed;
                if (video_accum_time >= video_frame_interval)
                {
                    video_writer.write(resize ? (cv::resize(image, image_resized, cv::Size(width_resized, height_resized)), image_resized) : image);
                    video_accum_time = 0.0;
                    last_video_frame_time = now;
                }
                else
                {
                    last_video_frame_time = now;
                }
            }

            static double filtered_x = 0.0, filtered_y = 0.0;

            if (is_detected)
            {
                if (!previous_detection)
                {
                    // Clear filtered values on rising edge of detection
                    filtered_x = x;
                    filtered_y = y;
                }
                filtered_x = alpha * x + (1 - alpha) * filtered_x;
                filtered_y = alpha * y + (1 - alpha) * filtered_y;
            }

            previous_detection = is_detected;

            auto message = dart_msgs::msg::GreenLight();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = camera_name;
            message.is_detected = is_detected;
            message.location.x = filtered_x;
            message.location.y = filtered_y;
            message.location.z = 0.0;
            greenlight_publisher_->publish(message);

            // 写Log
            if (is_detected)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Green light detection from %s camera: location=(%.2f, %.2f)",
                                     camera_name.c_str(), filtered_x, filtered_y);

            std_msgs::msg::Header header;
            header.stamp = this->now();

            auto image_msg = cv_bridge::CvImage(header, "bgr8", resize ? image_resized : image).toCompressedImageMsg();
            greenlight_image_publisher_->publish(*image_msg);

            auto end_time = std::chrono::steady_clock::now();
            auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            auto sleep_time = target_sleep_time_ms - processing_time;
            if (sleep_time > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time)));
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                     10000, "Greenlight processing time exceeded target frame time: %ld ms", processing_time);
            }
        }

        // 终止视频录制
        if (save_video && video_writer.isOpened())
        {
            video_writer.release();
            RCLCPP_INFO(this->get_logger(), "Video recording stopped for %s camera.", camera_name.c_str());
        }

        greenlight_publisher_->on_deactivate();
        greenlight_image_publisher_->on_deactivate();
    }
    RCLCPP_INFO(this->get_logger(), "Thread for %s camera stopped.", camera_name.c_str());
}

void NodeDartLauncherDetector::perform_greenlight_detection(cv::Mat &frame, bool &is_detected, double &x, double &y)
{
    if (frame.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Frame is empty, skipping detection");
        return;
    }
    if (greenlight_detector_->detect(frame))
    {
        is_detected = true;
        cv::Point2f center;
        greenlight_detector_->getResult(center);
        x = center.x;
        y = center.y;
        RCLCPP_DEBUG(this->get_logger(), "Green light detected: center at (%.2f, %.2f)", x, y);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "No green light detected in the current frame");
    }
    greenlight_detector_->drawRaw(frame);
}

void NodeDartLauncherDetector::on_parameter_event(const rclcpp::Parameter &param)
{
    if (param.get_name() == "lccv.enable")
    {
        lccv_enabled_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "LCCV camera enabled set to: %s", lccv_enabled_ ? "true" : "false");
    }
    else if (param.get_name() == "dh.enable")
    {
        dh_enabled_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "DH camera enabled set to: %s", dh_enabled_ ? "true" : "false");
    }
    else if (param.get_name() == "camera_watchdog.enable")
    {
        camera_watchdog_active_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Camera watchdog enabled set to: %s", camera_watchdog_active_ ? "true" : "false");

        // 如果启用看门狗，重置计数器
        if (camera_watchdog_active_)
        {
            lccv_failure_count_ = 0;
            dh_failure_count_ = 0;
            restart_attempts_ = 0;
        }
    }
    else if (param.get_name() == "camera_watchdog.failure_threshold")
    {
        int threshold = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Camera watchdog failure threshold set to: %d", threshold);
    }
}

NodeDartLauncherDetector::NodeDartLauncherDetector(rclcpp::NodeOptions options)
    : rclcpp_lifecycle::LifecycleNode("node_dart_launcher_detector", options),
      running_(false), lccv_enabled_(true), dh_enabled_(true),
      lccv_failure_count_(0), dh_failure_count_(0), restart_attempts_(0),
      camera_watchdog_active_(false), lccv_working_(false), dh_working_(false),
      led_files_initialized_(false)
{
    // 设置LED文件权限
    try
    {
        // 尝试为LED文件设置权限
        int chmod_result = 0;
        chmod_result = system("sudo chmod 666 /sys/class/leds/PWR/brightness");
        if (chmod_result != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set permissions for PWR LED: %s", strerror(errno));
        }

        chmod_result = system("sudo chmod 666 /sys/class/leds/ACT/brightness");
        if (chmod_result != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set permissions for ACT LED: %s", strerror(errno));
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception during LED permission setup: %s", e.what());
    }
}

// 新增一个通用函数，用于加载相机参数并打开相机
bool NodeDartLauncherDetector::load_and_open_camera(const std::string &camera_prefix, std::shared_ptr<CameraDriver> &camera_driver)
{
    RCLCPP_INFO(this->get_logger(), "Trying to open %s camera...", camera_prefix.c_str());

    // 获取相机参数
    std::unordered_map<std::string, std::string> camera_params;
    auto param_list = this->list_parameters({camera_prefix + ".camera_params"}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
    for (const auto &param : param_list.names)
    {
        // 截取参数名
        std::string param_name = param.substr(param.find_last_of(".") + 1);
        RCLCPP_INFO(this->get_logger(), "Parameter %s: %s", param_name.c_str(), this->get_parameter(param).value_to_string().c_str());

        if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE || this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            camera_params[param_name] = this->get_parameter(param).value_to_string();
        }
        else if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
            auto param_value = this->get_parameter(param).as_double_array();
            std::string param_value_str;
            for (const auto &value : param_value)
            {
                param_value_str += std::to_string(value) + " ";
            }
            camera_params[param_name] = param_value_str;
        }
        else if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            auto param_value = this->get_parameter(param).as_integer_array();
            std::string param_value_str;
            for (const auto &value : param_value)
            {
                param_value_str += std::to_string(value) + " ";
            }
            camera_params[param_name] = param_value_str;
        }
        else if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_STRING)
        {
            camera_params[param_name] = this->get_parameter(param).as_string();
        }
    }

    // 创建相机驱动实例并尝试打开
    if (camera_prefix == "lccv")
    {
        camera_driver = std::make_shared<CameraDriver_LCCV>();
    }
    else if (camera_prefix == "dh")
    {
        camera_driver = std::make_shared<CameraDriver_DH>();
    }

    if (!camera_driver->open(camera_params))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s camera with provided parameters.", camera_prefix.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "%s camera opened successfully.", camera_prefix.c_str());
    return true;
}

// 修改 on_configure 函数，调用通用函数
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartLauncherDetector::on_configure(
    const rclcpp_lifecycle::State &pre_state)
{
    RCLCPP_INFO(this->get_logger(), "Configuring node...");

    RCLCPP_INFO(this->get_logger(), "Loading parameters...");

    // 声明相机看门狗参数
    if (!this->has_parameter("camera_watchdog.enable"))
    {
        this->declare_parameter("camera_watchdog.enable", true);
    }

    if (!this->has_parameter("camera_watchdog.failure_threshold"))
    {
        this->declare_parameter("camera_watchdog.failure_threshold", 30);
    }

    camera_watchdog_active_ = this->get_parameter("camera_watchdog.enable").as_bool();
    RCLCPP_INFO(this->get_logger(), "Camera watchdog is %s", camera_watchdog_active_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "Camera watchdog failure threshold: %d", this->get_parameter("camera_watchdog.failure_threshold").as_int());

    // 必需参数检查
    const std::string required_params[] = {
        "lccv.enable",
        "lccv.image_width",
        "lccv.image_height",
        "dh.enable",
        "dh.image_width",
        "dh.image_height",
    };

    if (this->get_parameter("lccv.enable").get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
        lccv_enabled_ = this->get_parameter("lccv.enable").as_bool();
    }
    if (this->get_parameter("dh.enable").get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
        dh_enabled_ = this->get_parameter("dh.enable").as_bool();
    }

    for (const auto &param : required_params)
    {
        if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
            RCLCPP_ERROR(this->get_logger(), "Required parameter %s not set.", param.c_str());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
    }

    // 使用通用函数加载和打开相机
    if (lccv_enabled_)
    {
        if (!load_and_open_camera("lccv", camera_lccv_))
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        qr_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/dart_launcher_detector/image/qrcode", 10);
        qr_detect_publisher_ = this->create_publisher<std_msgs::msg::String>("/dart_launcher_detector/results/qrcode", 10);
    }

    if (dh_enabled_)
    {
        if (!load_and_open_camera("dh", camera_dh_))
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        greenlight_publisher_ = this->create_publisher<dart_msgs::msg::GreenLight>("/dart_launcher_detector/results/greenlight", 10);
        greenlight_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/dart_launcher_detector/image/greenlight_processed", 10);
    }

    // 初始化检测器，从参数中加载配置
    if (this->has_parameter("detect.greenlight_detect.parameter_file"))
    {
        RCLCPP_INFO(this->get_logger(), "Loading greenlight detector parameters from file...");
        std::string param_file = this->get_parameter("detect.greenlight_detect.parameter_file").as_string();
        greenlight_detector_ = std::make_shared<TopArmorDetect>(param_file);
    }
    else
    {
        if (this->has_parameter("detect.greenlight_detect.hmin"))
        {
            RCLCPP_INFO(this->get_logger(), "Loading greenlight detector parameters from node parameters...");
            int HMIN = this->get_parameter("detect.greenlight_detect.hmin").as_int();
            int HMAX = this->get_parameter("detect.greenlight_detect.hmax").as_int();
            int SMIN = this->get_parameter("detect.greenlight_detect.smin").as_int();
            int SMAX = this->get_parameter("detect.greenlight_detect.smax").as_int();
            int VMIN = this->get_parameter("detect.greenlight_detect.vmin").as_int();
            int VMAX = this->get_parameter("detect.greenlight_detect.vmax").as_int();
            double minDIST = this->get_parameter("detect.greenlight_detect.minDist").as_int();
            double rmin = this->get_parameter("detect.greenlight_detect.rmin").as_int();
            double rmax = this->get_parameter("detect.greenlight_detect.rmax").as_int();
            double PARAM1 = this->get_parameter("detect.greenlight_detect.param1").as_int();
            double PARAM2 = this->get_parameter("detect.greenlight_detect.param2").as_int();

            greenlight_detector_ = std::make_shared<TopArmorDetect>(HMIN, HMAX, SMIN, SMAX, VMIN, VMAX, minDIST, rmin, rmax, PARAM1, PARAM2);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Greenlight detector parameter file not set and no parameters provided.");
            greenlight_detector_ = std::make_shared<TopArmorDetect>();
        }
    }

    if (!greenlight_detector_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize greenlight detector.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(), "Greenlight detector initialized successfully.");

    try
    {

        RCLCPP_INFO(this->get_logger(), "Adding parameter event callback...");

        callback_set_parameter_handle = this->add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult
            {
                for (const auto &param : params)
                {
                    RCLCPP_INFO(this->get_logger(), "Parameter update: %s", param.get_name().c_str());
                    on_parameter_event(param);
                }
                // 如果处于激活状态，则重新配置节点
                if (this->get_current_state().label() == "active")
                {
                    RCLCPP_INFO(this->get_logger(), "Reconfiguring node due to parameter change...");
                    this->deactivate();
                    this->cleanup();
                    this->configure();
                    this->activate();
                }
                else if (this->get_current_state().label() == "inactive")
                {
                    RCLCPP_INFO(this->get_logger(), "Node not active, reconfiguring...");
                    this->cleanup();
                    this->configure();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Node is not active and no reconfiguration needed.");
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    RCLCPP_INFO(this->get_logger(), "Configuration complete: Cameras, callbacks, publishers and detectors initialized.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NodeDartLauncherDetector::on_activate(
    const rclcpp_lifecycle::State &pre_state)
{
    RCLCPP_INFO(this->get_logger(), "Activating node...");

    // 重置看门狗计数器和工作状态
    lccv_failure_count_ = 0;
    dh_failure_count_ = 0;
    restart_attempts_ = 0;
    lccv_working_ = false;
    dh_working_ = false;
    running_ = true;

    if (lccv_enabled_)
    {
        if (camera_lccv_->isOpened)
        {
            RCLCPP_INFO(this->get_logger(), "LCCV camera is opened.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "LCCV camera is not opened. Please reconfigure.");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        lccv_thread_ = std::make_shared<std::thread>(std::bind(&NodeDartLauncherDetector::camera_thread, this, camera_lccv_, "lccv", true));
        RCLCPP_INFO(this->get_logger(), "LCCV camera thread started.");
    }
    if (dh_enabled_)
    {
        if (camera_dh_->isOpened)
        {
            RCLCPP_INFO(this->get_logger(), "DH camera is opened.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "DH camera is not opened.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        dh_thread_ = std::make_shared<std::thread>(std::bind(&NodeDartLauncherDetector::camera_thread, this, camera_dh_, "dh", false));
        RCLCPP_INFO(this->get_logger(), "DH camera thread started.");
    }

    // 初始化LED文件和状态
    if (initialize_led_files())
    {
        set_led_state("PWR", false);
        set_led_state("ACT", true);

        // 启动LED控制线程
        led_control_thread_ = std::make_shared<std::thread>(&NodeDartLauncherDetector::led_control_thread_function, this);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize LED files, LED control will not be available");
    }
    RCLCPP_INFO(this->get_logger(), "LED control thread started.");

    RCLCPP_INFO(this->get_logger(), "Node activation complete.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartLauncherDetector::on_deactivate(
    const rclcpp_lifecycle::State &pre_state)
{
    RCLCPP_INFO(this->get_logger(), "Deactivating node: Stopping threads...");
    running_ = false;

    if (lccv_thread_)
    {
        if (!lccv_thread_->joinable())
            RCLCPP_WARN(this->get_logger(), "LCCV camera thread is not joinable.");
        lccv_thread_->join();
        RCLCPP_INFO(this->get_logger(), "LCCV camera thread joined.");
    }

    if (dh_thread_)
    {
        if (!dh_thread_->joinable())
            RCLCPP_WARN(this->get_logger(), "DH camera thread is not joinable.");
        dh_thread_->join();
        RCLCPP_INFO(this->get_logger(), "DH camera thread joined.");
    }

    RCLCPP_INFO(this->get_logger(), "All camera threads stopped.");

    // 停止LED控制线程
    if (led_control_thread_)
    {
        if (led_control_thread_->joinable())
        {
            led_control_thread_->join();
            RCLCPP_INFO(this->get_logger(), "LED control thread joined.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "LED control thread is not joinable.");
        }
    }

    // 确保LED恢复正常状态
    set_led_state("PWR", false); // 默认电源灭
    set_led_state("ACT", false); // 默认活动灯亮

    // 关闭LED文件，下次激活时重新打开
    close_led_files();

    RCLCPP_INFO(this->get_logger(), "Node deactivation complete.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartLauncherDetector::on_cleanup(
    const rclcpp_lifecycle::State &pre_state)
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up resources...");
    camera_lccv_->close();
    camera_dh_->close();
    camera_lccv_.reset();
    camera_dh_.reset();
    greenlight_publisher_.reset();
    qr_image_publisher_.reset();
    qr_detect_publisher_.reset();
    greenlight_image_publisher_.reset();
    greenlight_detector_.reset();

    // 关闭LED文件
    close_led_files();
    RCLCPP_INFO(this->get_logger(), "Resources successfully cleaned up.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeDartLauncherDetector::on_shutdown(
    const rclcpp_lifecycle::State &pre_state)
{
    RCLCPP_INFO(this->get_logger(), "Shutting down node...");
    running_ = false;

    // 停止所有线程
    if (lccv_thread_)
    {
        if (lccv_thread_->joinable())
        {
            lccv_thread_->join();
        }
        lccv_thread_.reset();
    }

    if (dh_thread_)
    {
        if (dh_thread_->joinable())
        {
            dh_thread_->join();
        }
        dh_thread_.reset();
    }

    if (led_control_thread_)
    {
        if (led_control_thread_->joinable())
        {
            led_control_thread_->join();
        }
        led_control_thread_.reset();
    }

    // 确保LED恢复正常状态
    set_led_state("PWR", false); // 默认电源灯灭
    set_led_state("ACT", false); // 默认活动灯亮
    // 关闭LED文件
    close_led_files();

    RCLCPP_INFO(this->get_logger(), "Node shutdown complete.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void NodeDartLauncherDetector::set_led_state(const std::string &led, bool state)
{
    // 如果文件未初始化，则尝试初始化
    if (!led_files_initialized_ && !initialize_led_files())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize LED files when setting LED state");
        return;
    }

    try
    {
        std::ofstream *led_file = nullptr;
        if (led == "PWR")
        {
            led_file = pwr_led_file_.get();
        }
        else if (led == "ACT")
        {
            led_file = act_led_file_.get();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown LED: %s", led.c_str());
            return;
        }

        if (led_file && led_file->is_open())
        {
            // 在写入前移动到文件开头
            led_file->seekp(0);
            *led_file << (state ? "1" : "0");
            led_file->flush();
            RCLCPP_DEBUG(this->get_logger(), "Set LED %s state to %d", led.c_str(), state ? 1 : 0);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "LED file for %s is not valid", led.c_str());
            // 尝试重新初始化文件
            close_led_files();
            if (initialize_led_files())
            {
                set_led_state(led, state); // 递归调用一次
            }
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception when setting LED %s state: %s", led.c_str(), e.what());
        // 出现异常时尝试重新初始化
        close_led_files();
        initialize_led_files();
    }
}

void NodeDartLauncherDetector::led_control_thread_function()
{
    RCLCPP_INFO(this->get_logger(), "LED control thread started");
    bool pwr_state = false;
    bool act_state = true;

    // 确保LED文件已经初始化
    if (!led_files_initialized_ && !initialize_led_files())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize LED files, LED control thread exiting");
        return;
    }

    while (running_ && rclcpp::ok())
    {
        if (!led_files_initialized_)
        {
            // 如果文件句柄丢失，尝试重新初始化
            if (!initialize_led_files())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                     5000, "Could not initialize LED files, retrying...");
                std::this_thread::sleep_for(1000ms);
                continue;
            }
        }

        try
        {
            if (lccv_working_ && dh_working_)
            {
                // 交替闪烁LED
                pwr_state = !pwr_state;
                act_state = !act_state;

                set_led_state("PWR", pwr_state);
                set_led_state("ACT", act_state);
            }
            else
            {
                // 如果相机异常，设置固定状态
                set_led_state("PWR", false);
                set_led_state("ACT", true);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in LED control thread: %s", e.what());
            // 发生错误时尝试重新初始化文件
            close_led_files();
        }

        // 每500毫秒切换一次LED状态
        std::this_thread::sleep_for(500ms);
    }

    // 退出时恢复LED默认状态
    if (led_files_initialized_)
    {
        set_led_state("PWR", false); // 默认电源灭
        set_led_state("ACT", false); // 默认活动灯亮
    }

    RCLCPP_INFO(this->get_logger(), "LED control thread stopped");
}

void NodeDartLauncherDetector::try_restart_node()
{
    std::lock_guard<std::mutex> lock(restart_mutex_);

    // 如果已经尝试重启三次，则转为inactive状态
    if (restart_attempts_ >= 3)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to restart node after 3 attempts, switching to inactive state");

        // 切换到inactive状态
        if (this->get_current_state().label() == "active")
        {
            this->deactivate();
        }

        restart_attempts_ = 0;
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Camera watchdog triggered. Attempting to restart node (attempt %d/3)", restart_attempts_.load() + 1);
    restart_attempts_++;

    // 类似于参数回调中的重启逻辑
    if (this->get_current_state().label() == "active")
    {
        RCLCPP_INFO(this->get_logger(), "Reconfiguring node due to camera failure...");
        this->deactivate();
        this->cleanup();
        this->configure();
        this->activate();
    }
    else if (this->get_current_state().label() == "inactive")
    {
        RCLCPP_INFO(this->get_logger(), "Node not active, reconfiguring...");
        this->cleanup();
        this->configure();
    }
}

bool NodeDartLauncherDetector::initialize_led_files()
{
    if (led_files_initialized_)
    {
        return true; // 已经初始化过了
    }

    try
    {
        // 打开PWR LED文件
        pwr_led_file_ = std::make_unique<std::ofstream>("/sys/class/leds/PWR/brightness");
        if (!pwr_led_file_->is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open PWR LED file");
            return false;
        }

        // 打开ACT LED文件
        act_led_file_ = std::make_unique<std::ofstream>("/sys/class/leds/ACT/brightness");
        if (!act_led_file_->is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open ACT LED file");
            pwr_led_file_->close();
            pwr_led_file_.reset();
            return false;
        }

        led_files_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "LED files successfully initialized");
        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception during LED file initialization: %s", e.what());
        close_led_files();
        return false;
    }
}

void NodeDartLauncherDetector::close_led_files()
{
    if (pwr_led_file_ && pwr_led_file_->is_open())
    {
        pwr_led_file_->close();
        pwr_led_file_.reset();
    }

    if (act_led_file_ && act_led_file_->is_open())
    {
        act_led_file_->close();
        act_led_file_.reset();
    }

    led_files_initialized_ = false;
    RCLCPP_INFO(this->get_logger(), "LED files closed");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().use_intra_process_comms(false);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<NodeDartLauncherDetector>(options);
    RCLCPP_INFO(node->get_logger(), "Node started. Spinning...");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}