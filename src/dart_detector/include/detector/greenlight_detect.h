#ifndef DETECT_H
#define DETECT_H

#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

class TopArmorDetect
{
public:
    TopArmorDetect(const std::string &paramFile = "./src/dart_detector/config/config_greenlight.csv");
    TopArmorDetect(int HMIN, int HMAX, int SMIN, int SMAX, int VMIN, int VMAX, double minDIST, double rmin, double rmax, double PARAM1, double PARAM2);

    void loadParameters(const std::string &paramFile);
    void Hough_Circle(cv::Mat &inputImage);
    void select();
    void preprocess(cv::Mat &frame, cv::Mat &result);

    bool detect(cv::Mat &frame);

    cv::Mat drawResult();
    cv::Mat debugDraw();
    cv::Mat drawRaw();
    void getResult(cv::Point2f &center);

private:
    int HMIN, // 色相下界
        HMAX, // 色相上界
        SMIN, // 饱和度下界
        SMAX, // 饱和度上界
        VMIN, // 色调下界
        VMAX; // 色调（亮度）上界

    double minDIST, // 检测到的圆的中心之间的最小距离
        RMIN,       // 最小半径
        RMAX,       // 最大半径
        PARAM1,
        PARAM2;
    /*
        param1：它是传递给 cv::Canny 边缘检测算法的高阈值。边缘检测阶段会使用这个值来确定边缘强度。
        如果你设置一个较低的值，将会检测到更多的边缘，可能包括噪声；
        如果设置一个较高的值，将会减少检测到的边缘，但可能会漏掉一些较弱的边缘。

        param2：它是圆检测阶段的累加器阈值。当 Hough 圆检测算法检测到足够多的边缘像素时，会判断它是否是一个圆。
        较低的 param2 值可以使算法检测到更多的圆，但这些圆可能是虚假的。
        较高的 param2 值使得算法更严格，只检测到非常明显的圆。
    */
    cv::Point2f _center;
    float _radius;
    std::vector<cv::Vec3f> _circles;
    cv::Mat _preprocessResult;
    cv::Mat _rawImg;
};

#endif // DETECT_H
