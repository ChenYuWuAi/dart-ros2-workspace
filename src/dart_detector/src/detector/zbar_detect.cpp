#include "detector/zbar_detect.h"

QRCodeDetectorZB::QRCodeDetectorZB()
{
    // 初始化zbar扫描器，仅启用二维码识别
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

QRCodeDetectorZB::~QRCodeDetectorZB() {}

void QRCodeDetectorZB::setBinaryThreshold(double threshold)
{
    binary_threshold = threshold;
}

std::vector<std::string> QRCodeDetectorZB::detect(cv::Mat &inputImage)
{
    std::vector<std::string> results;

    if (inputImage.empty())
    {
        return results; // 返回空结果
    }

    // 转换为灰度图
    cv::Mat gray, filtered, claheImg;
    cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);

    // 1.1 双边滤波保边去噪
    cv::bilateralFilter(gray, filtered, 9, 75, 75);

    // 在二值化之前，先做一次中值滤波去小颗粒噪声
    cv::medianBlur(filtered, claheImg, 5);

    // 1.3 自适应阈值二值化
    cv::adaptiveThreshold(
        claheImg, claheImg, 255,
        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY, binary_threshold, 5);

    // 形态学开运算去除噪点
    // 开运算去掉小白点
    cv::Mat opened;
    cv::morphologyEx(claheImg, opened, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, {3, 3}),
                     cv::Point(-1, -1), 1);

    inputImage = opened.clone();

    // 将处理后的图像转换为zbar所需的格式
    zbar::Image imagez(opened.cols, opened.rows, "Y800", opened.data, opened.cols * opened.rows);
    scanner.scan(imagez);

    // 遍历检测到的二维码
    for (zbar::Image::SymbolIterator symbol = imagez.symbol_begin(); symbol != imagez.symbol_end(); ++symbol)
    {
        if (symbol->get_type() == zbar::ZBAR_QRCODE)
        {
            results.push_back(symbol->get_data());
        }
    }

    return results;
}