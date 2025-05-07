/**
 * @file camera_dh.cpp
 * @note 大华相机HAL驱动实现
 */
#include <DHCameraDriver/DHVideoCapture.h>
#include <camera_hal/camera_dh.hpp>

using namespace CameraHAL;

CameraDriver_DH::CameraDriver_DH()
{
    _videoCapture = new DHVideoCapture();
}

CameraDriver_DH::~CameraDriver_DH()
{
    delete _videoCapture;
}

#define IS_PARAM_SET(param) (params.find(param) != params.end())

bool CameraDriver_DH::open(std::unordered_map<std::string, std::string> &params)
{
    int id = 0;
    int size_buffer = 2;

    if (isOpened)
    {
        return true;
    }

    if(!_videoCapture->discovery())
    {
        return false;
    }

    if (IS_PARAM_SET("id"))
    {
        id = std::stoi(params["id"]);
    }
    if (IS_PARAM_SET("size_buffer"))
    {
        size_buffer = std::stoi(params["size_buffer"]);
    }

    if (!_videoCapture->open(id, size_buffer))
    {
        return false;
    }

    isOpened = true;

    // 初始化相机参数
    for (auto &param : params)
    {
        write(param.first, param.second);
    }

    _videoCapture->startStream();
    _videoCapture->closeStream();

    _videoCapture->startStream();

    return true;
}

bool CameraDriver_DH::write(std::string para_name, std::string para_value)
{
    if (!isOpened)
    {
        return false;
    }
    return _videoCapture->write(para_name, para_value);
}

bool CameraDriver_DH::read(cv::Mat &image)
{
    if (!isOpened)
    {
        return false;
    }
    return _videoCapture->read(image);
}

bool CameraDriver_DH::close()
{
    if(!isOpened)
    {
        return true;
    }

    if (!_videoCapture->closeStream())
    {
        return false;
    }

    isOpened = false;

    return true;
}