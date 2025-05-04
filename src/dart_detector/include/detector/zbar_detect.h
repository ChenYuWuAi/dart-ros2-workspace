#ifndef QRCODE_DETECTOR_H
#define QRCODE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <vector>
#include <string>

class QRCodeDetectorZB
{
public:
    QRCodeDetectorZB();
    ~QRCodeDetectorZB();

    /**
     * @brief 检测输入图像中的二维码
     * @param inputImage 输入的OpenCV图像 (cv::Mat)
     * @return 检测到的二维码数据列表 (std::vector<std::string>)
     */
    std::vector<std::string> detect(cv::Mat &inputImage);

    /**
     * @brief 设置二值化阈值
     * @param threshold 二值化阈值
     */
    void setBinaryThreshold(double threshold);

private:
    zbar::ImageScanner scanner;
    double binary_threshold = 200.0; // 二值化阈值
};

#endif // QRCODE_DETECTOR_H