/**
 * @file camera_lccv.cpp
 * @note 树莓派LCCV相机HAL驱动实现
 */
#include <camera_hal/camera_lccv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <string>
namespace CameraHAL
{
    CameraDriver_LCCV::CameraDriver_LCCV()
    {
        isOpened = false;
    }

    CameraDriver_LCCV::~CameraDriver_LCCV()
    {
        close();
    }

    bool CameraDriver_LCCV::open(std::unordered_map<std::string, std::string> &params)
    {
        // 获取分辨率和帧率参数，设置默认值
        int width = 640, height = 480, framerate = 30;

        // 配置 LCCV 相机参数
        camera.options->video_width = width;
        camera.options->video_height = height;
        camera.options->framerate = framerate;

        for (const auto &param : params)
        {
            write(param.first, param.second);
        }
        camera.options->verbose = true;

        // 启动视频流
        try
        {
            camera.startVideo();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to start LCCV camera: " << e.what() << std::endl;
            return false;
        }

        isOpened = true;
        return true;
    }

    bool CameraDriver_LCCV::write(std::string para_name, std::string para_value)
    {
        std::cout << "Setting parameter: " << para_name << " = " << para_value << std::endl;
        if (para_name == "Width")
        {
            camera.options->video_width = std::stoi(para_value);
        }
        else if (para_name == "Height")
        {
            camera.options->video_height = std::stoi(para_value);
        }
        else if (para_name == "Resolution")
        {
            // 用isstream读取数字 需要两个参数 width height 用逗号分隔！
            std::istringstream iss(para_value);
            int width, height;
            iss >> width;
            if (iss)
            {
                iss >> height;
            }
            else
            {
                std::cout << "failed to parse string to width!";
            }
            if (iss.fail())
            {
                "Error:expected string transformation";
                return false;
            }
            camera.options->video_width = width;
            camera.options->video_height = height;
        }
        else if (para_name == "Framerate")
        {
            camera.options->framerate = std::stoi(para_value);
        }
        else if (para_name == "ExposureMode")
        {
            if (para_value == "auto")
            {
                camera.options->setExposureMode(EXPOSURE_NORMAL);
            }
            else if (para_value == "short")
            {
                camera.options->setExposureMode(EXPOSURE_SHORT);
            }
            else if (para_value == "custom")
            {
                camera.options->setExposureMode(EXPOSURE_CUSTOM);
            }
            else
            {
                std::cerr << "Unsupported exposure mode: " << para_value << std::endl;
                return false;
            }
        }
        else if (para_name == "ExposureTime")
        {
            camera.options->shutter = std::stof(para_value);
        }
        else if (para_name == "Brightness")
        {
            camera.options->brightness = std::stof(para_value);
        }
        else if (para_name == "Contrast")
        {
            camera.options->contrast = std::stof(para_value);
        }
        else if (para_name == "Saturation")
        {
            camera.options->saturation = std::stof(para_value);
        }
        else if (para_name == "Sharpness")
        {
            camera.options->sharpness = std::stof(para_value);
        }
        else
        {
            std::cerr << "Unsupported parameter: " << para_name << std::endl;
            return false;
        }

        return true;
    }

    bool CameraDriver_LCCV::read(cv::Mat &image)
    {
        if (!isOpened)
        {
            std::cerr << "Camera is not opened" << std::endl;
            return false;
        }

        // 获取视频帧
        if (!camera.getVideoFrame(image, 1000))
        {
            std::cerr << "Failed to capture video frame" << std::endl;
            return false;
        }

        return true;
    }

    bool CameraDriver_LCCV::close()
    {
        if (!isOpened)
        {
            return true;
        }

        // 停止视频流
        try
        {
            camera.stopVideo();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to stop LCCV camera: " << e.what() << std::endl;
            return false;
        }

        std::cout << "LCCV camera stopped successfully." << std::endl;

        isOpened = false;

        return true;
    }
} // namespace CameraHAL