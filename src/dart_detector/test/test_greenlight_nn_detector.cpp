#include <iostream>
#include <opencv2/opencv.hpp>
#include "detector/greenlight_nn_detect.hpp"
#include <chrono>
#include <string>

class GreenlightNnDetectorTest
{
public:
    GreenlightNnDetectorTest(const std::string &modelPath, const std::string &weightsPath, const cv::Size2d &inputSize, float confThreshold, float nmsThreshold)
        : detector(modelPath, weightsPath, inputSize, confThreshold, nmsThreshold) {}

    void runOnce(const std::string &imagePath, bool gui)
    {
        cv::Mat img = cv::imread(imagePath);
        if (img.empty())
        {
            std::cerr << "Error: Unable to load image: " << imagePath << std::endl;
            return;
        }

        auto results = detector.detect(img);

        if (results.empty())
        {
            std::cout << "No objects detected." << std::endl;
        }
        std::cout << "Detected " << results.size() << " objects." << std::endl;

        for (const auto &result : results)
        {
            std::cout << "Detected box: " << result.box << ", score: " << result.score << ", center: " << result.center << std::endl;
            if (gui)
            {
                cv::rectangle(img, result.box, cv::Scalar(0, 255, 0), 2);
                cv::putText(img, std::to_string(result.score), result.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
        }

        if (gui)
        {
            cv::imshow("Detection Results", img);
            cv::waitKey(0);
        }
    }

    void runFpsTest(const std::string &imagePath, int iterations)
    {
        cv::Mat img = cv::imread(imagePath);
        if (img.empty())
        {
            std::cerr << "Error: Unable to load image: " << imagePath << std::endl;
            return;
        }

        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; ++i)
        {
            detector.detect(img);
        }
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> elapsed = end - start;
        double fps = iterations / elapsed.count();
        std::cout << "FPS Test: " << fps << " frames per second over " << iterations << " iterations." << std::endl;
    }

private:
    NnDetector detector;
};

int main(int argc, char **argv)
{
    std::string modelPath = "src/dart_detector/config/greenlight_nn.xml";
    std::string weightsPath = "src/dart_detector/config/greenlight_nn.bin";
    cv::Size2d inputSize(640, 384);
    float confThreshold = 0.25;
    float nmsThreshold = 0.1;
    std::string imagePath;
    bool gui = false;
    bool once = false;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--once")
        {
            once = true;
        }
        else if (arg == "--image" && i + 1 < argc)
        {
            imagePath = argv[++i];
        }
        else if (arg == "--conf" && i + 1 < argc)
        {
            confThreshold = std::stof(argv[++i]);
        }
        else if (arg == "--nms" && i + 1 < argc)
        {
            nmsThreshold = std::stof(argv[++i]);
        }
        else if (arg == "--gui")
        {
            gui = true;
        }
    }

    if (!imagePath.empty())
    {
        GreenlightNnDetectorTest test(modelPath, weightsPath, inputSize, confThreshold, nmsThreshold);
        if (once)
        {
            test.runOnce(imagePath, false);
            test.runOnce(imagePath, gui);
        }
        else
        {
            test.runFpsTest(imagePath, 100);
        }
    }
    else
    {
        std::cerr << "Usage: " << argv[0] << " [--once] --image <image_path> [--conf <confidence_threshold>] [--nms <nms_threshold>] [--gui]" << std::endl;
    }

    return 0;
}