#include <node_launcher_detector.hpp>

using namespace std::chrono_literals;
using namespace CameraHAL;

void NodeDartLauncherDetector::camera_thread(std::shared_ptr<CameraDriver> camera, const std::string &camera_name, bool is_qr_detection)
{
    RCLCPP_INFO(this->get_logger(), "Starting thread for %s camera...", camera_name.c_str());

    int width = this->get_parameter(camera_name + ".image_width").as_int();
    int height = this->get_parameter(camera_name + ".image_height").as_int();
    RCLCPP_DEBUG(this->get_logger(), "%s camera resolution: %dx%d", camera_name.c_str(), width, height);

    cv::Mat image(height, width, CV_8UC3);
    if (is_qr_detection)
    {
        // Activate publishers
        qr_image_publisher_->on_activate();
        qr_detect_publisher_->on_activate();

        std::string last_detected_qr_code_str;
        int reset_last_detected_qr_code_counter = 0;
        while (running_ && rclcpp::ok())
        {
            if (!camera->read(image))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to read image from %s camera", camera_name.c_str());
                std::this_thread::sleep_for(16ms);
                continue;
            }
            RCLCPP_DEBUG(this->get_logger(), "Image read successfully from %s camera", camera_name.c_str());

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
            std::this_thread::sleep_for(16ms);
        }
        qr_image_publisher_->on_deactivate();
        qr_detect_publisher_->on_deactivate();
    }
    else
    {

        greenlight_publisher_->on_activate();
        greenlight_image_publisher_->on_activate();
        while (running_ && rclcpp::ok())
        {
            try
            {
                if (!camera->read(image))
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to read image from %s camera", camera_name.c_str());
                    std::this_thread::sleep_for(16ms);
                    continue;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception while reading image from %s camera: %s", camera_name.c_str(), e.what());
                std::this_thread::sleep_for(16ms);
                continue;
            }

            RCLCPP_DEBUG(this->get_logger(), "Image read successfully from %s camera", camera_name.c_str());

            bool is_detected = false;
            double x = 0, y = 0;
            if (image.empty())
                continue;
            cv::Mat resultImg = perform_greenlight_detection(image, is_detected, x, y);
            if (is_detected)
            {
                RCLCPP_INFO(this->get_logger(), "%s camera detected green light at (%.2f, %.2f)", camera_name.c_str(), x, y);
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "No green light detected in current frame of %s camera", camera_name.c_str());
            }

            auto message = dart_msgs::msg::GreenLight();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = camera_name;
            message.is_detected = is_detected;
            message.location.x = x;
            message.location.y = y;
            message.location.z = 0.0;
            greenlight_publisher_->publish(message);

            std_msgs::msg::Header header;
            header.stamp = this->now();

            auto image_msg = cv_bridge::CvImage(header, "bgr8", resultImg).toCompressedImageMsg();
            greenlight_image_publisher_->publish(*image_msg);
        }

        greenlight_publisher_->on_deactivate();
        greenlight_image_publisher_->on_deactivate();
    }
    RCLCPP_INFO(this->get_logger(), "Thread for %s camera stopped.", camera_name.c_str());
}

cv::Mat NodeDartLauncherDetector::perform_greenlight_detection(cv::Mat &frame, bool &is_detected, double &x, double &y)
{
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
    return greenlight_detector_->drawRaw();
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
}

NodeDartLauncherDetector::NodeDartLauncherDetector(rclcpp::NodeOptions options)
    : rclcpp_lifecycle::LifecycleNode("dart_launcher_detector", options),
      running_(false), lccv_enabled_(true), dh_enabled_(true)
{
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

    // 设置QR码检测器参数
    if (this->has_parameter("detect.qr_detect.binThreshold"))
    {
        int binThreshold = this->get_parameter("detect.qr_detect.binThreshold").as_int();
        RCLCPP_INFO(this->get_logger(), "QR Code detector binary threshold set to: %d", binThreshold);
        qr_detector_.setBinaryThreshold(binThreshold);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "QR code detector binary threshold not set, using default value 200.");
        qr_detector_.setBinaryThreshold(200);
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
        RCLCPP_INFO(this->get_logger(), "LCCV camera thread started and detached.");
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
        RCLCPP_INFO(this->get_logger(), "DH camera thread started and detached.");
    }
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

    RCLCPP_INFO(this->get_logger(), "All threads stopped.");

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
    RCLCPP_INFO(this->get_logger(), "Resources successfully cleaned up.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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