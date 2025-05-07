#include <arpa/inet.h>
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/StreamSource.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "Infra/PrintLog.h"
#include "Memory/SharedPtr.h"
#include "Infra/CString.h"
#include "GenICam/ParameterNode.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "DHVideoCapture.h"
#include <chrono>

using namespace std;
using namespace cv;
using namespace Dahua::GenICam;
using namespace Dahua::Infra;

bool DHVideoCapture::discovery()
{
    bool isDiscoverySuccess = _systemObj.discovery(_vCameraPtrList);
    if (!isDiscoverySuccess)
    {
        return false;
    }
    if (_vCameraPtrList.size() == 0)
    {
        return false;
    }
    return true;
}

DHVideoCapture::DHVideoCapture()
{
    bool isDiscoverySuccess = _systemObj.discovery(_vCameraPtrList);
    if (!isDiscoverySuccess)
    {
        printf("discovery device fail.\n");
    }
    if (_vCameraPtrList.size() == 0)
    {
        printf("no camera.\n");
    }
    _resolution = new int[2];
    _resolution[0] = _resolution[1] = 0;
    _exposure_time = 0;
    _fps = 0;
    _gamma = 0;
    _gain = 0;
    _balance_ratio = new double[3];
    for (int i = 0; i < 3; ++i)
    {
        _balance_ratio[i] = 0;
    }
}

DHVideoCapture::~DHVideoCapture()
{
    if (_cameraSptr)
        if (!_cameraSptr->disConnect())
        {
            printf("disConnect camera failed\n");
        }
    delete[] _resolution;
    delete[] _balance_ratio;
}

bool DHVideoCapture::open(const int id, int size_buffer)
{
    if (!_vCameraPtrList[id]->connect())
    {
        printf("connect cameral failed.\n");
        return false;
    }
    _cameraSptr = _vCameraPtrList[id];

    _streamPtr = _systemObj.createStreamSource(_cameraSptr);
    _streamPtr.reset();
    _streamPtr = _systemObj.createStreamSource(_cameraSptr);
    if (NULL == _streamPtr)
    {
        printf("create stream obj  fail.\r\n");
        return false;
    }
    bool isSettingSuccess = _streamPtr->setBufferCount(size_buffer);
    if (!isSettingSuccess)
    {
        printf("Setting  fail.\n");
        return false;
    }
    _camera_id = id;
    return true;
}

bool DHVideoCapture::write(string para_name, string para_value)
{
    std::cout << "write para_name: " << para_name << " para_value: " << para_value << std::endl;
    if (para_name == "Resolution")
    {
        // 用isstream读取数字 需要两个参数 width height 用逗号分隔！
        istringstream iss(para_value);
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

        return setVideoFormat(width, height);
    }
    else if (para_name == "ExposureTime")
    {
        int exposure_time = std::stoi(para_value);
        return setExposureTime(exposure_time);
    }
    else if (para_name == "FPS")
    {
        double fps = std::stod(para_value);
        return setFPS(fps);
    }
    else if (para_name == "BalanceRatio")
    {
        // 用isstream读取数字 需要四个参数 red green blue 用逗号分隔！
        istringstream iss(para_value);
        double red, green, blue;
        bool autoBalance;
        iss >> autoBalance;
        if (iss)
        {
            iss >> red;
        }
        else
        {
            std::cout << "failed to parse string to red!";
        }
        if (iss)
        {
            iss >> green;
        }
        else
        {
            std::cout << "failed to parse string to green!";
        }
        if (iss)
        {
            iss >> blue;
        }
        else
        {
            std::cout << "failed to parse string to blue!";
        }
        if (iss.fail())
        {
            "Error:expected string transformation";
            return false;
        }
        return setBalanceRatio(red, green, blue, autoBalance);
    }
    else if (para_name == "Gamma")
    {
        double gamma = std::stod(para_value);
        return setGamma(gamma);
    }
    else if (para_name == "Gain")
    {
        double gain = std::stod(para_value);
        return setGain(gain);
    }
}

bool DHVideoCapture::isOpened()
{
    return _cameraSptr->isConnected();
}

void DHVideoCapture::info()
{

    CStringNode paramDeviceVersion(_cameraSptr, "DeviceVersion");
    CString strDeviceVersion;
    paramDeviceVersion.getValue(strDeviceVersion);

    printf("Camera[%d] Info :\n", _camera_id);
    printf("    key           = [%s]\n", _cameraSptr->getKey());
    printf("    vendor name   = [%s]\n", _cameraSptr->getVendorName());
    printf("    model         = [%s]\n", _cameraSptr->getModelName());
    printf("    serial number = [%s]\n", _cameraSptr->getSerialNumber());
    printf("    DeviceVersion = [%s]\n", strDeviceVersion.c_str());

    //_cameraSptr = _vCameraPtrList[_camera_id];
    printf("    Resolution    = [%d X %d]\n", _resolution[0], _resolution[1]);
    printf("    FPS           = [%f]\n", _fps);
    printf("    Exposure time = [%dms]\n", _exposure_time);
    printf("    Balance ratio:\n\
    Red           = [%f]\n\
    Green         = [%f]\n\
    Blue          = [%f]\n",
           _balance_ratio[0], _balance_ratio[1], _balance_ratio[2]);
    printf("    Gamma         = [%f]\n", _gamma);
    printf("    Brightness    = [%f]\n", _gain);
}

bool DHVideoCapture::startStream()
{
    bool isStartGrabbingSuccess = _streamPtr->startGrabbing(0, IStreamSource::EGrabStrategy::grabStrartegyLatestImage);

    if (!isStartGrabbingSuccess)
    {
        printf("StartGrabbing  fail.\n");
        return false;
    }
    else
    {
        return true;
    }
}

bool DHVideoCapture::setVideoFormat(size_t width, size_t height, bool mjpg)
{
    if (width == _resolution[0] && height == _resolution[1])
        return true;
    int32_t bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(_cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return false;
    }

    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.setValue(width);
    if (false == bRet)
    {
        printf("set width fail.\n");
        return false;
    }

    intNode = sptrImageFormatControl->height();
    bRet = intNode.setValue(height);
    if (false == bRet)
    {
        printf("set height fail.\n");
        return false;
    }

    /* OffsetX */
    intNode = sptrImageFormatControl->offsetX();
    bRet = intNode.setValue((1280 - width) / 2);
    if (!bRet)
    {
        printf("set offsetX fail.\n");
        return false;
    }

    /* OffsetY */
    intNode = sptrImageFormatControl->offsetY();
    bRet = intNode.setValue((1024 - height) / 2);
    if (!bRet)
    {
        printf("set offsetY fail.\n");
        return false;
    }

    _resolution[0] = width;
    _resolution[1] = height;
    return true;
}

bool DHVideoCapture::setExposureTime(int t)
{
    // if(t == _exposure_time)
    //     return true;
    bool bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(_cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return false;
    }
    if (t == 0)
    {
        CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
        bRet = enumNode.setValueBySymbol("Continuous");
        if (false == bRet)
        {
            printf("set exposureAuto fail.\n");
            return false;
        }
    }
    else
    {
        CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
        bRet = enumNode.setValueBySymbol("Off");
        if (false == bRet)
        {
            printf("set exposureAuto fail.\n");
            return false;
        }
        t *= 1000;
        CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
        bRet = doubleNode.setValue(t);
        if (false == bRet)
        {
            printf("set exposureTime fail.\n");
            return false;
        }
        _exposure_time = t / 1000;
    }
    return true;
}

// void DHVideoCapture::getFPS()
//{
//     cout<<"fps is now "<<_fps<<endl;

//}
int DHVideoCapture::getAcquisitionFrameRate(double &dFrameRate)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(_cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.getValue(dFrameRate);
    if (false == bRet)
    {
        printf("get acquisitionFrameRate fail.\n");
        return -1;
    }
    return 0;
}
bool DHVideoCapture::setFPS(double fps)
{
    if (fps == _fps)
        return true;

    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(_cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return false;
    }

    CBoolNode booleanNode = sptAcquisitionControl->acquisitionFrameRateEnable();
    bRet = booleanNode.setValue(true);
    if (false == bRet)
    {
        printf("set acquisitionFrameRateEnable fail step 1.\n");
        return false;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.setValue(fps);
    if (false == bRet)
    {
        printf("set acquisitionFrameRate fail.step 2\n");
        return false;
    }
    return true;
}

bool DHVideoCapture::setBalanceRatio(double red, double green, double blue, bool autoBalance)
{
    if (red == _balance_ratio[0] && green == _balance_ratio[1] && blue == _balance_ratio[2])
        return true;
    //    auto t1 = chrono::high_resolution_clock::now();
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(_cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return false;
    }

    /* 关闭自动白平衡 */
    CEnumNode enumNode = sptrAnalogControl->balanceWhiteAuto();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return false;
    }

    if (autoBalance == true)
    {
        bRet = enumNode.setValueBySymbol("Continuous");
        if (false == bRet)
        {
            printf("set balanceWhiteAuto On(Continuous) fail.\n");
            return false;
        }
        _balance_ratio[0] = _balance_ratio[1] = _balance_ratio[2] = -1;
    }
    else
    {
        bRet = enumNode.setValueBySymbol("Off");
        if (false == bRet)
        {
            printf("set balanceWhiteAuto Off fail.\n");
            return false;
        }

        CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
        bRet = doubleNode.setValue(red);
        if (false == bRet)
        {
            printf("set red balanceRatio fail.\n");
            return false;
        }
        _balance_ratio[0] = red;

        enumNode = sptrAnalogControl->balanceRatioSelector();
        bRet = enumNode.setValueBySymbol("Green");
        if (false == bRet)
        {
            printf("set green balanceRatioSelector fail.\n");
            return false;
        }

        doubleNode = sptrAnalogControl->balanceRatio();
        bRet = doubleNode.setValue(green);
        if (false == bRet)
        {
            printf("set green balanceRatio fail.\n");
            return false;
        }
        _balance_ratio[1] = green;

        enumNode = sptrAnalogControl->balanceRatioSelector();
        bRet = enumNode.setValueBySymbol("Blue");
        if (false == bRet)
        {
            printf("set blue balanceRatioSelector fail.\n");
            return false;
        }

        doubleNode = sptrAnalogControl->balanceRatio();
        bRet = doubleNode.setValue(blue);
        if (false == bRet)
        {
            printf("set blue balanceRatio fail.\n");
            return false;
        }
        _balance_ratio[2] = blue;
    }
    //    auto t2 = chrono::high_resolution_clock::now();
    //    cout << "Capture period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;

    return true;
}

bool DHVideoCapture::getBalanceRatio()
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(_cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return false;
    }

    CEnumNode enumNode = sptrAnalogControl->balanceRatioSelector();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = enumNode.setValueBySymbol("Red");
    if (false == bRet)
    {
        printf("set red balanceRatioSelector fail.\n");
        return false;
    }

    CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(_balance_ratio[0]);
    if (false == bRet)
    {
        printf("get red balanceRatio fail.\n");
        return false;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Green");
    if (false == bRet)
    {
        printf("set green balanceRatioSelector fail.\n");
        return false;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(_balance_ratio[1]);
    if (false == bRet)
    {
        printf("get green balanceRatio fail.\n");
        return false;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Blue");
    if (false == bRet)
    {
        printf("set blue balanceRatioSelector fail.\n");
        return false;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(_balance_ratio[2]);
    if (false == bRet)
    {
        printf("get blue balanc_quit_flageRatio fail.\n");
        return false;
    }
    return true;
}

bool DHVideoCapture::setGamma(double gamma)
{
    if (gamma == _gamma)
        return true;
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(_cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return false;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gamma();
    bRet = doubleNode.setValue(gamma);
    if (false == bRet)
    {
        printf("set gamma fail.\n");
        return false;
    }
    _gamma = gamma;
    return true;
}

bool DHVideoCapture::setGain(double gain)
{
    if (gain == _gain)
        return true;

    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(_cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return false;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
    bRet = doubleNode.setValue(gain);
    if (false == bRet)
    {
        printf("get gainRaw fail.\n");
        return false;
    }
    _gain = gain;
    return true;
}

bool DHVideoCapture::changeVideoFormat(int width, int height, bool mjpg)
{
    closeStream();
    restartCapture();
    setVideoFormat(width, height, mjpg);
    startStream();
    return true;
}

bool DHVideoCapture::grab(CFrame &frame)
{
    if (!_streamPtr)
    {
        printf("_streamPtr is NULL.\n");
        return false;
    }

    _grab_done = _streamPtr->getFrame(frame); // 500
    if (!_grab_done)
    {
        grab_failed_time++;
        cout << "grab_failed_time:  " << grab_failed_time << endl;
        printf("getFrame  fail.\n");
        return false;
    }
    else
    {
        if (grab_failed_time > 0)
            grab_failed_time--;
    }

    bool isValid = frame.valid();
    if (!isValid)
    {
        printf("frame is invalid!\n");
        return false;
    }
    return true;
}

bool DHVideoCapture::retrieve(Mat &image, CFrame &frame)
{
    const void *frameData = frame.getImage();
    Mat raw((_resolution[1]), (_resolution[0]), CV_8UC1, const_cast<void *>(frameData));
    cvtColor(raw, image, COLOR_BayerBG2BGR);
    frame.reset();
    return true;
}

bool DHVideoCapture::read(Mat &image)
{
    CFrame frame;
    //    StreamStatisticsInfo ss;
    //    this->_streamPtr->getStatisticsInfo(ss);
    //    cout<<ss.pcieStatisticsInfo.bandwidth<<"  sad\n";
    grab(frame);
    retrieve(image, frame);
    return !image.empty();
}

bool DHVideoCapture::closeStream()
{
    bool isStopGrabbingSuccess = _streamPtr->stopGrabbing();
    if (!isStopGrabbingSuccess)
    {
        printf("StopGrabbing  fail.\n");
        return false;
    }
    else
    {
        return true;
    }
}

cv::Size DHVideoCapture::getResolution()
{
    return cv::Size(_resolution[0], _resolution[1]);
}

bool DHVideoCapture::restartCapture()
{
    if (_cameraSptr->disConnect())
    {
        printf("(restart)camera disconnect failed.\n");
        return false;
    }
    if (!_cameraSptr->connect())
    {
        printf("camera restart failed.\n");
        return false;
    }
    _resolution[0] = _resolution[1] = 0;
    _exposure_time = 0;
    _fps = 0;
    _gamma = 0;
    _gain = 0;
    for (int i = 0; i < 3; ++i)
    {
        _balance_ratio[i] = 0;
    }
    return true;
}
