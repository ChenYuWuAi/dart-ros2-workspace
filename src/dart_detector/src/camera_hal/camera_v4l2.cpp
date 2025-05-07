/**
 * @file camera_v4l2.cpp
 * @note V4L2相机HAL驱动实现
 */
#include <unordered_map>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <camera_hal/camera_v4l2.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

namespace CameraHAL
{

    CameraDriver_V4L2::CameraDriver_V4L2()
    {
    }

    CameraDriver_V4L2::~CameraDriver_V4L2()
    {
        close();
    }

    bool CameraDriver_V4L2::open(std::unordered_map<std::string, std::string> &params)
    {
        // 获取相机路径，默认为 /dev/video0
        std::string video_device = "/dev/video0";
        if (params.find("DevicePath") != params.end())
        {
            video_device = params["DevicePath"];
        }

        // 设置视频格式，默认为 SRGGB10_1X10 格式，1280x720 分辨率
        // 执行初始化命令
        std::string cmd1 = "media-ctl -d /dev/media0 --set-v4l2 '\"m00_b_imx219 2-0010\":0[fmt:SRGGB10_1X10/1280x720]'";
        std::string cmd2 = "media-ctl -d /dev/media0 --set-v4l2 '\"rkisp-isp-subdev\":0[fmt:SRGGB10_1X10/1280x720]'";
        std::string cmd3 = "media-ctl -d /dev/media0 --set-v4l2 '\"rkisp-isp-subdev\":0[crop:(0,0)/1280x720]'";

        if (system(cmd1.c_str()) != 0)
        {
            printf("Failed to execute media-ctl commands\n");
            return false;
        }

        // 打开视频设备
        fd = ::open(video_device.c_str(), O_RDWR);
        if (fd == -1)
        {
            printf("Opening video device %s failed\n", video_device.c_str());
            return false;
        }

        // 改为多平面类型
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width = 1280;
        fmt.fmt.pix_mp.height = 720;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;

        if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
        {
            perror("Setting format");
            ::close(fd);
            return false;
        }

        req.count = BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1)
        {
            perror("Requesting buffers");
            ::close(fd);
            return false;
        }

        for (int i = 0; i < req.count; ++i)
        {
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            buf.length = 1; // 多平面时需设置为平面数，例如 1

            // planes 用来存储每个平面的信息
            struct v4l2_plane planes[VIDEO_MAX_PLANES];
            memset(planes, 0, sizeof(planes));
            buf.m.planes = planes;

            if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1)
            {
                perror("Querying buffer");
                ::close(fd);
                return false;
            }

            // 对每个平面进行 mmap
            buffer_start[i] = mmap(
                NULL,
                buf.m.planes[0].length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd,
                buf.m.planes[0].m.mem_offset);

            if (buffer_start == MAP_FAILED)
            {
                perror("Memory mapping buffer");
                ::close(fd);
                return false;
            }

            if (ioctl(fd, VIDIOC_QBUF, &buf) == -1)
            {
                perror("Queueing buffer");
                ::close(fd);
                return false;
            }
        }

        // 设置参数
        isOpened = true;

        for (auto &param : params)
        {
            write(param.first, param.second);
        }

        // 开始视频流捕捉
        if (ioctl(fd, VIDIOC_STREAMON, &fmt.type) == -1)
        {
            perror("Starting video stream");
            ::close(fd);
            return false;
        }

        return true;
    }

    bool CameraDriver_V4L2::write(std::string para_name, std::string para_value)
    {
        if (!isOpened)
        {
            std::cerr << "Camera is not opened" << std::endl;
            return false;
        }

        // 创建 v4l2_control 结构体
        struct v4l2_control ctrl;

        // 根据参数名和传入值设置控制项
        if (para_name == "Exposure")
        {
            // 这里使用硬编码的 0x00980911
            ctrl.id = 0x00980911;
            ctrl.value = std::stoi(para_value);
        }
        else if (para_name == "Gain")
        {
            ctrl.id = 0x00980913;
            ctrl.value = std::stoi(para_value);
        }
        else if (para_name == "AnalogueGain")
        {
            ctrl.id = 0x009e0903;
            ctrl.value = std::stoi(para_value); // 参数值转换为整数
        }
        else
        {
            return false;
        }

        // 使用 ioctl 设置控制项
        if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1)
        {
            std::string err_msg = "Setting control " + para_name + " failed";
            perror(err_msg.c_str());
            return false;
        }

        return true;
    }

    bool CameraDriver_V4L2::read(cv::Mat &image)
    {
        if (!isOpened)
        {
            return false;
        }

        struct v4l2_plane planes[1]; // 定义多平面缓冲区
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1; // 多平面缓冲区需要设置为最大平面数

        // 取出缓冲区
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1)
        {
            perror("Dequeueing buffer failed");
            return false;
        }

        // 创建 YUV 格式的图像
        cv::Mat frame(fmt.fmt.pix_mp.height,
                      fmt.fmt.pix_mp.width,
                      CV_8UC2,
                      buffer_start[buf.index],                   // 使用 mmap 得到的指针
                      fmt.fmt.pix_mp.plane_fmt[0].bytesperline); // 每行的字节数

        // 转换为 BGR 格式
        cv::Mat bgr_frame;
        cv::cvtColor(frame, bgr_frame, cv::COLOR_YUV2BGR_UYVY);

        // 将结果复制到输出参数
        bgr_frame.copyTo(image);

        // 将缓冲区重新排队
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1)
        {
            perror("Requeueing buffer failed");
            return false;
        }

        return true;
    }

    bool CameraDriver_V4L2::close()
    {
        if (!isOpened)
        {
            return true;
        }

        // 停止视频流捕捉
        if (ioctl(fd, VIDIOC_STREAMOFF, &fmt.type) == -1)
        {
            perror("Stopping video stream");
            return false;
        }

        // 解除内存映射
        for (int i = 0; i < req.count; ++i)
        {
            munmap(buffer_start, buf.length);
        }

        // 关闭视频设备
        if (::close(fd) == -1)
        {
            perror("Closing video device");
            return false;
        }

        isOpened = false;
        return true;
    }
} // namespace CameraHAL