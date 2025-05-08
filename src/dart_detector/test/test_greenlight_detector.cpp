#include <iostream>
#include <opencv2/opencv.hpp>
#include "detector/greenlight_detect.h"
#include <chrono>

class GreenlightDetectorTest {
public:
    void runTests() {
        testDetectEmptyImage();
        testDetectValidTarget();
        testPerformance();
    }

    TopArmorDetect detector{"./src/dart_detector/config/config_greenlight.csv"}; // Ensure TopArmorDetect is correctly defined

    void testDetectEmptyImage() {
        std::cout << "Running test: DetectEmptyImage" << std::endl;
        cv::Mat emptyImage;
        bool result = detector.detect(emptyImage);
        if (!result) {
            std::cout << "Test passed: No targets detected in empty image." << std::endl;
        } else {
            std::cerr << "Test failed: Targets detected in empty image." << std::endl;
        }
    }

    void testDetectValidTarget() {
        std::cout << "Running test: DetectValidTarget" << std::endl;
        cv::Mat targetImage = cv::imread("src/dart_detector/test/sample_target_picture.png");
        if (targetImage.empty()) {
            std::cerr << "Test failed: Unable to load target image." << std::endl;
            return;
        }

        bool result = detector.detect(targetImage);
        if (result) {
            cv::Point2f center;
            detector.getResult(center);
            std::cout << "Test passed: Target detected at (" << center.x << ", " << center.y << ")." << std::endl;
        } else {
            std::cerr << "Test failed: Target not detected." << std::endl;
        }
    }

    void testPerformance() {
        std::cout << "Running test: Performance" << std::endl;
        cv::Mat targetImage = cv::imread("src/dart_detector/test/sample_target_picture.png");
        if (targetImage.empty()) {
            std::cerr << "Test failed: Unable to load target image." << std::endl;
            return;
        }

        const int iterations = 100;
        auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < iterations; ++i) {
            detector.detect(targetImage);
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        double fps = iterations / elapsed.count();
        std::cout << "Performance test completed: " << fps << " FPS." << std::endl;
    }
};

int main() {
    GreenlightDetectorTest testSuite;
    testSuite.runTests();
    return 0;
}