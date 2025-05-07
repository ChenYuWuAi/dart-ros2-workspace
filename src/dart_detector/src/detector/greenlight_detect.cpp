#include "detector/greenlight_detect.h"

/**
 * @brief 构造函数，从paramFile读取参数
 */
TopArmorDetect::TopArmorDetect(const std::string &paramFile)
{
    loadParameters(paramFile);
}

TopArmorDetect::TopArmorDetect(int HMIN, int HMAX, int SMIN, int SMAX, int VMIN, int VMAX, double minDIST, double rmin, double rmax, double PARAM1, double PARAM2) : HMIN(HMIN), HMAX(HMAX), SMIN(SMIN), SMAX(SMAX), VMIN(VMIN), VMAX(VMAX), minDIST(minDIST), RMIN(rmin), RMAX(rmax), PARAM1(PARAM1), PARAM2(PARAM2)
{
}

/**
 * @brief 读取参数文件
 *
 * @param paramFile 文件地址
 */
void TopArmorDetect::loadParameters(const std::string &paramFile)
{
    std::ifstream file(paramFile);
    if (!file.is_open())
    {
        std::cerr << "check config path: " << paramFile << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string key;
        int value;
        if (std::getline(iss, key, ',') && iss >> value)
        {
            if (key == "hmin")
                HMIN = value;
            else if (key == "hmax")
                HMAX = value;
            else if (key == "smin")
                SMIN = value;
            else if (key == "smax")
                SMAX = value;
            else if (key == "vmin")
                VMIN = value;
            else if (key == "vmax")
                VMAX = value;
            else if (key == "minDist")
                minDIST = value;
            else if (key == "rmin")
                RMIN = value;
            else if (key == "rmax")
                RMAX = value;
            else if (key == "param1")
                PARAM1 = value;
            else if (key == "param2")
                PARAM2 = value;
        }
    }

    file.close();
}

/**
 * @brief 霍夫曼变换找圆
 *
 * @param inputImage 输入图像
 */
void TopArmorDetect::Hough_Circle(cv::Mat &inputImage)
{
    cv::Mat gray, dst;
    cv::cvtColor(inputImage, dst, cv::COLOR_BGR2GRAY);
    // cv::GaussianBlur(gray, dst, cv::Size(9, 9), 2, 2);

    cv::HoughCircles(dst, _circles, cv::HOUGH_GRADIENT, 1, minDIST, PARAM1, PARAM2, 0, 0);

    // std::cout << "circles_size(): " << _circles.size() << std::endl;
}

/**
 * @brief 多个圆筛选
 *
 */
void TopArmorDetect::select()
{
    _center = cv::Point2f(0.0f, 0.0f);
    _radius = 0.0f;
    if (_circles.empty())
        return;
    for (auto &circle : _circles)
    {
        if (circle[2] >= RMAX || circle[2] <= RMIN)
            continue;
        _center += cv::Point2f(circle[0], circle[1]);
        _radius += circle[2];
    }
    _center.x /= _circles.size();
    _center.y /= _circles.size();
    _radius /= _circles.size();
}

/**
 * @brief hsv 从原图筛选出绿色部分
 *
 * @param frame 原图
 * @param result 结果
 */
void TopArmorDetect::preprocess(cv::Mat &frame, cv::Mat &result)
{
    cv::Mat src;
    cv::cvtColor(frame, src, cv::COLOR_BGR2HSV);

    cv::Scalar lower_green(HMIN, SMIN, VMIN); // 绿色的下界（H, S, V）
    cv::Scalar upper_green(HMAX, SMAX, VMAX); // 绿色的上界（H, S, V）

    cv::Mat mask;
    cv::inRange(src, lower_green, upper_green, mask);
    cv::bitwise_and(frame, frame, result, mask);
    _preprocessResult = result;
}

/**
 * @brief 完整预测流程
 *
 * @param frame 原图
 */
bool TopArmorDetect::detect(cv::Mat &frame)
{
    _circles.clear();
    _rawImg = frame.clone();
    cv::Mat result;
    preprocess(frame, result);
    try
    {
        Hough_Circle(result);
    }
    catch (cv::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
    select();
    return _circles.size() > 0;
}

/**
 * @brief 绘制结果
 *
 */
cv::Mat TopArmorDetect::drawResult()
{
    cv::Mat draw = _preprocessResult.clone();

    cv::circle(draw, _center, 3, cv::Scalar(0, 0, 255), -1);
    cv::circle(draw, _center, _radius, cv::Scalar(0, 0, 255), 3);
    // text .1f
    static char buf[50];
    sprintf(buf, "(%.1f, %.1f)", _center.x, _center.y);
    cv::putText(draw, std::string(buf), cv::Point(_center.x, _center.y + 2 * _radius), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);

    return draw;
}

/**
 * @brief 绘制结果,调参用
 */
cv::Mat TopArmorDetect::debugDraw()
{
    cv::Mat draw = _preprocessResult.clone();
    for (auto circle : _circles)
    {
        cv::circle(draw, cv::Point2f(circle[0], circle[1]), 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(draw, cv::Point2f(circle[0], circle[1]), circle[2], cv::Scalar(0, 0, 255), 3);
    }
    return draw;
}

cv::Mat TopArmorDetect::drawRaw()
{
    cv::Mat draw = _rawImg.clone();

    cv::circle(draw, _center, 3, cv::Scalar(0, 0, 255), -1);
    cv::circle(draw, _center, _radius, cv::Scalar(0, 0, 255), 3);
    static char buf[50];
    sprintf(buf, "(%.1f, %.1f)", _center.x, _center.y);
    cv::putText(draw, std::string(buf), cv::Point(_center.x, _center.y + 2 * _radius), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);

    return draw;
}
void TopArmorDetect::getResult(cv::Point2f &center)
{
    center = _center;
}
