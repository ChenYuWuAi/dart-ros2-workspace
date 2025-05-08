#pragma once
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <map>

struct NnResult
{
    cv::Rect box;
    float score;
    cv::Point2f center;
};

class NnDetector
{
public:
    // path: XML/BIN/DEVICE paths; 设置 input_size、score_threshold、nms_threshold
    NnDetector(const std::string &path_xml, const std::string &path_bin,
               const cv::Size2d &input_size,
               float score_threshold,
               float nms_threshold);
    std::vector<NnResult> detect(const cv::Mat &frame);
    cv::Mat draw(const cv::Mat &frame, const std::vector<NnResult> &res);

    std::vector<float> getResult(const std::vector<NnResult> &res_);

    // 预处理输出尺寸及偏移
    struct Resize
    {
        cv::Mat resized_image;
        int dx;
        int dy;
    };
    // 双缓冲推理
    static constexpr bool CURR = true, NEXT = false;
    NnDetector::Resize letterBox(cv::Mat &src, const cv::Size2d &dst);
    void fitRec(std::vector<NnResult> &b, cv::Size2d ori, cv::Size2d now);
    void setScoreThreshold(float score_threshold);
    void setNmsThreshold(float nms_threshold);

private:
    std::string xml_, bin_, device_;
    ov::Core core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_model_;
    ov::CompiledModel compiled_model_next_;
    std::map<bool, ov::InferRequest> infer_requests_;
    cv::Size2d input_size_;
    float score_threshold_;
    float nms_threshold_;
    bool first_startup_ = true;
};