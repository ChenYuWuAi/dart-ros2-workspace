#include "detector/qrcode_detect.h"
#include <opencv2/wechat_qrcode.hpp>

QRCodeDetectorWechat::QRCodeDetectorWechat()
{
    // // 初始化zbar扫描器，仅启用二维码识别
    // scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    // 初始化WeChat QRCode检测器
    try
    {
        wechat_detector = cv::makePtr<cv::wechat_qrcode::WeChatQRCode>(
            "src/dart_detector/thirdparty/wechat_qrcode/detect.prototxt", "src/dart_detector/thirdparty/wechat_qrcode/detect.caffemodel",
            "src/dart_detector/thirdparty/wechat_qrcode/sr.prototxt", "src/dart_detector/thirdparty/wechat_qrcode/sr.caffemodel");
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to initialize WeChatQRCode: " << e.what() << std::endl;
    }
}

QRCodeDetectorWechat::~QRCodeDetectorWechat() {}

void QRCodeDetectorWechat::setBinaryThreshold(double threshold)
{
    binary_threshold = threshold;
}

std::vector<std::string> QRCodeDetectorWechat::detect(cv::Mat &inputImage)
{
    std::vector<std::string> results;

    if (inputImage.empty())
    {
        return results; // 返回空结果
    }

    // 转换为灰度图
    // cv::Mat gray, filtered, claheImg;
    // cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);

    // // 1.1 双边滤波保边去噪
    // cv::bilateralFilter(gray, filtered, 9, 75, 75);

    // // 在二值化之前，先做一次中值滤波去小颗粒噪声
    // cv::medianBlur(filtered, claheImg, 5);

    // // 1.3 自适应阈值二值化
    // cv::adaptiveThreshold(
    //     claheImg, claheImg, 255,
    //     cv::ADAPTIVE_THRESH_GAUSSIAN_C,
    //     cv::THRESH_BINARY, binary_threshold, 5);

    // // 形态学开运算去除噪点
    // cv::Mat opened;
    // cv::morphologyEx(claheImg, opened, cv::MORPH_OPEN,
    //                  cv::getStructuringElement(cv::MORPH_RECT, {3, 3}),
    //                  cv::Point(-1, -1), 1);

    // inputImage = opened.clone();

    // 使用WeChat QRCode检测器
    std::vector<cv::Mat> points;
    try
    {
        auto wechat_results = wechat_detector->detectAndDecode(inputImage, points);
        results.insert(results.end(), wechat_results.begin(), wechat_results.end());
    }
    catch (const std::exception &e)
    {
        std::cerr << "WeChat QRCode detection failed: " << e.what() << std::endl;
    }

    return results;
}