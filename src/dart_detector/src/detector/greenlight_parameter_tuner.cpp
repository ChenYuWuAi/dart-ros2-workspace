/*
调参用

 g++ ./Parameters.cpp ./detect.cpp -o parameters $(pkg-config --cflags --libs opencv4)
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include "detector/greenlight_detect.h"

using namespace cv;
using namespace std;

// 输入图像
Mat img;
VideoCapture cap;
// HSV图像
Mat hsv;
// 色相
int hmin = 1;
int hmin_Max = 360;
int hmax = 180;
int hmax_Max = 180;
// 饱和度
int smin = 1;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
// 亮度
int vmin = 1;
int vmin_Max = 255;
int vmax = 255;
int vmax_Max = 255;
// 最小圆心距离
int minDist = 1;
int minDist_Max = 400;
// 半径
int rmin = 1;
int rmin_Max = 400;
int rmax = 1;
int rmax_Max = 400;
// param1
int param1 = 1;
int param1_Max = 50;
// param2
int param2 = 1;
int param2_Max = 50;

string dstName = "dst";

void saveParameters(const string &filename = "./src/dart_detector/config/config.csv")
{
    ofstream file(filename);
    if (file.is_open())
    {
        file << "hmin," << hmin << endl;
        file << "hmax," << hmax << endl;
        file << "smin," << smin << endl;
        file << "smax," << smax << endl;
        file << "vmin," << vmin << endl;
        file << "vmax," << vmax << endl;
        file << "minDist," << minDist << endl;
        file << "rmin," << rmin << endl;
        file << "rmax," << rmax << endl;
        file << "param1," << param1 << endl;
        file << "param2," << param2 << endl;
        file.close();
        cout << "Parameters saved to " << filename << endl;
    }
    else
    {
        cerr << "Error opening file for writing: " << filename << endl;
    }
}

void loadParameters(const std::string &paramFile = "./src/dart_detector/config/config.csv")
{
    ifstream file(paramFile);
    if (!file.is_open())
    {
        cerr << "check config path: " << paramFile << std::endl;
        return;
    }

    string line;
    while (std::getline(file, line))
    {
        istringstream iss(line);
        string key;
        int value;
        if (getline(iss, key, ',') && iss >> value)
        {
            if (key == "hmin")
                hmin = value;
            else if (key == "hmax")
                hmax = value;
            else if (key == "smin")
                smin = value;
            else if (key == "smax")
                smax = value;
            else if (key == "vmin")
                vmin = value;
            else if (key == "vmax")
                vmax = value;
            else if (key == "minDist")
                minDist = value;
            else if (key == "rmin")
                rmin = value;
            else if (key == "rmax")
                rmax = value;
            else if (key == "param1")
                param1 = value;
            else if (key == "param2")
                param2 = value;
        }
    }

    file.close();
}

void callBack(int, void *)
{
    if (hmin == 0 || hmax == 0 || smin == 0 || smax == 0 || vmin == 0 || vmax == 0 || rmin == 0 || rmax == 0 || minDist == 0 || param1 == 0 || param2 == 0)
    {
        return;
    }
    TopArmorDetect detector(hmin, hmax, smin, smax, vmin, vmax, minDist, rmin, rmax, param1, param2);

    detector.detect(img);
    Mat dst = detector.drawResult();

    imshow(dstName, dst);
}

int main(int argc, char **argv)
{
    if (argc == 2)
    {
        string input = argv[1];
        if (input.find(".jpg") != string::npos || input.find(".png") != string::npos)
        {
            img = imread(input);
            if (img.empty())
            {
                cerr << "Check img path" << endl;
                return -1;
            }
        }
        else
        {
            cap.open(input);
            cap >> img;
            if (!cap.isOpened())
            {
                cerr << "Check video path" << endl;
                return -1;
            }
        }
    }
    else
    {
        cerr << "Usage: " << argv[0] << " <video_file>/or/<img_file>" << std::endl;
        return -1;
    }

    // 颜色空间转换
    if (!img.empty())
    {
        cvtColor(img, hsv, COLOR_BGR2HSV);
    }

    // 定义输出图像的显示窗口
    namedWindow(dstName, WINDOW_GUI_EXPANDED);

    loadParameters();

    createTrackbar("hmin", dstName, &hmin, hmin_Max, callBack);
    createTrackbar("hmax", dstName, &hmax, hmax_Max, callBack);
    createTrackbar("smin", dstName, &smin, smin_Max, callBack);
    createTrackbar("smax", dstName, &smax, smax_Max, callBack);
    createTrackbar("vmin", dstName, &vmin, vmin_Max, callBack);
    createTrackbar("vmax", dstName, &vmax, vmax_Max, callBack);
    createTrackbar("minDist", dstName, &minDist, minDist_Max, callBack);
    createTrackbar("rmin", dstName, &rmin, rmin_Max, callBack);
    createTrackbar("rmax", dstName, &rmax, rmax_Max, callBack);
    createTrackbar("param1", dstName, &param1, param1_Max, callBack);
    createTrackbar("param2", dstName, &param2, param2_Max, callBack);

    cout << "\n'n' to go to the next frame. \n's' to save the parameters to a CSV file.\nESC to exit." << endl;

    while (true)
    {
        int key = waitKey(0);
        if (key == 's')
        {
            saveParameters();
            break;
        }
        else if (key == 'n')
        {
            if (cap.isOpened())
            {
                cap >> img;
                if (img.empty())
                {
                    saveParameters();
                    break;
                }
                callBack(0, 0);
            }
            else
            {
                cout << "to end" << endl;
            }
        }
        else if (key == 27)
        { // ESC
            break;
        }
    }

    destroyAllWindows();
    return 0;
}
