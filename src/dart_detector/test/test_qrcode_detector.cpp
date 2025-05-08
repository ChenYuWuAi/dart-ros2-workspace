#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "detector/qrcode_detect.h"

class QRCodeDetectorTest {
public:
    void runTests() {
        testDetectEmptyImage();
        testDetectValidQRCode();
        testDetectionFrameRate();
    }

private:
    QRCodeDetector detector;

    void testDetectEmptyImage() {
        std::cout << "Running test: DetectEmptyImage" << std::endl;
        cv::Mat emptyImage;
        auto results = detector.detect(emptyImage);
        if (results.empty()) {
            std::cout << "Test passed: No QR codes detected in empty image." << std::endl;
        } else {
            std::cerr << "Test failed: QR codes detected in empty image." << std::endl;
        }
    }

    void testDetectValidQRCode() {
        std::cout << "Running test: DetectValidQRCode" << std::endl;
        cv::Mat qrCodeImage = cv::imread("src/dart_detector/test/sample_qrcode.png");
        if (qrCodeImage.empty()) {
            std::cerr << "Test failed: Unable to load QR code image." << std::endl;
            return;
        }

        auto results = detector.detect(qrCodeImage);
        std::cout << "Detected QR codes: " << results.size() << std::endl;
        for (const auto &result : results) {
            std::cout << "QR code data: " << result << std::endl;
        }

        if (!results.empty()) { 
            std::cout << "Test passed: Valid QR code detected." << std::endl;
        } else {
            std::cerr << "Test failed: Valid QR code not detected or data mismatch." << std::endl;
        }
    }

    void testDetectionFrameRate() {
        std::cout << "Running test: DetectionFrameRate" << std::endl;
        cv::Mat qrCodeImage = cv::imread("src/dart_detector/test/sample_qrcode.png");
        if (qrCodeImage.empty()) {
            std::cerr << "Test failed: Unable to load QR code image." << std::endl;
            return;
        }

        const int numIterations = 100;
        auto startTime = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < numIterations; ++i) {
            detector.detect(qrCodeImage);
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedTime = endTime - startTime;

        double fps = numIterations / elapsedTime.count();
        std::cout << "Detection frame rate: " << fps << " FPS" << std::endl;

        if (fps > 0) {
            std::cout << "Test passed: Frame rate measured successfully." << std::endl;
        } else {
            std::cerr << "Test failed: Frame rate measurement failed." << std::endl;
        }
    }
};

int main() {
    QRCodeDetectorTest testSuite;
    testSuite.runTests();
    return 0;
}