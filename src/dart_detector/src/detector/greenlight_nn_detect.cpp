#include "detector/greenlight_nn_detect.hpp"
#define BLUE 0
#define RED 1
#define NONE_ 2
#define CURR true
#define NEXT false

NnDetector::NnDetector(const std::string &path_xml, const std::string &path_bin,
                       const cv::Size2d &input_size,
                       float score_threshold,
                       float nms_threshold)
    : xml_(path_xml), bin_(path_bin), device_("CPU"),
      input_size_(input_size),
      score_threshold_(score_threshold),
      nms_threshold_(nms_threshold)
{
    // 读模型并设置预处理
    model_ = core_.read_model(xml_, bin_);
    ov::preprocess::PrePostProcessor ppp(model_);
    ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
    ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({255.f, 255.f, 255.f});
    ppp.input().model().set_layout("NCHW");
    for (int i = 0; i < 3; ++i)
        ppp.output(i).tensor().set_element_type(ov::element::f32);
    model_ = ppp.build();
    // 编译双模型和双 InferRequest
    compiled_model_ = core_.compile_model(model_, device_);
    compiled_model_next_ = core_.compile_model(model_, device_);
    infer_requests_[CURR] = compiled_model_.create_infer_request();
    infer_requests_[NEXT] = compiled_model_next_.create_infer_request();
}

void NnDetector::setScoreThreshold(float score_threshold){
    score_threshold_ = score_threshold;
}

void NnDetector::setNmsThreshold(float nms_threshold)
{
    nms_threshold_ = nms_threshold;
}

std::vector<NnResult> NnDetector::detect(const cv::Mat &src)
{
    if (src.empty())
        return {};
    // letterBox 预处理
    Resize r = letterBox(const_cast<cv::Mat &>(src), input_size_);
    uint8_t *in_data = (uint8_t *)r.resized_image.data;
    ov::Tensor in_t = ov::Tensor(
        compiled_model_.input().get_element_type(),
        compiled_model_.input().get_shape(),
        in_data);
    // 首次启动
    if (first_startup_)
    {
        infer_requests_[CURR].set_input_tensor(in_t);
        infer_requests_[CURR].start_async();
        first_startup_ = false;
        return {};
    }
    // 正常推理双缓冲
    infer_requests_[NEXT].set_input_tensor(in_t);
    infer_requests_[NEXT].start_async();
    infer_requests_[CURR].wait();
    // 解析输出0
    const ov::Tensor &out = infer_requests_[CURR].get_output_tensor(0);
    auto shape = out.get_shape();
    float *pdata = out.data<float>();
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<NnResult> lights;
    for (size_t i = 0; i < shape[1]; ++i)
    {
        float *det = pdata + i * shape[2];
        float conf = det[4];
        if (conf < score_threshold_)
            continue;
        float *clsConf = det + 5;
        cv::Mat sc(1, 9, CV_32F, clsConf);
        double maxc;
        cv::Point maxp;
        cv::minMaxLoc(sc, nullptr, &maxc, nullptr, &maxp);
        if (maxc < score_threshold_ || maxp.x != 8)
            continue;
        float cx = det[0], cy = det[1], w = det[2], h = det[3];
        cv::Rect box(cx - w / 2, cy - h / 2, w, h);
        lights.push_back({box, conf, {cx, cy}});
        boxes.push_back(box);
        scores.push_back(conf);
    }
    // NMS
    std::vector<int> idx;
    cv::dnn::NMSBoxes(boxes, scores, score_threshold_, nms_threshold_, idx);
    std::vector<NnResult> res;
    for (int i : idx)
        res.push_back(lights[i]);
    // swap requests
    std::swap(infer_requests_[CURR], infer_requests_[NEXT]);
    // 坐标反变换
    fitRec(res, cv::Size2d(src.size()), cv::Size2d(input_size_));
    return res;
}

cv::Mat NnDetector::draw(const cv::Mat &frame, const std::vector<NnResult> &res)
{
    cv::Mat out = frame.clone();
    for (auto &r : res)
    {
        cv::rectangle(out, r.box, {0, 255, 0}, 2);
        char buf[32];
        sprintf(buf, "%.2f", r.score);
        cv::putText(out, buf, r.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
    }
    return out;
}

std::vector<float> NnDetector::getResult(const std::vector<NnResult> &res)
{
    if (res.empty())
        return {};
    // 取第一个
    cv::Point2f center = res[0].center;
    std::cout << "NnDetector center: " << center.x << ", " << center.y << std::endl;
    // 返回 x, y
    std::vector<float> result;
    for (auto &r : res)
    {
        result.push_back(r.center.x);
        result.push_back(r.center.y);
    }
    return result;
}

// letterBox 实现
NnDetector::Resize NnDetector::letterBox(cv::Mat &src, const cv::Size2d &dst)
{
    int w = src.cols, h = src.rows;
    double rw = dst.width / w, rh = dst.height / h;
    double r = std::min(rw, rh);
    int nw = w * r, nh = h * r;
    cv::Mat rs;
    cv::resize(src, rs, cv::Size(nw, nh));
    cv::Mat canvas(dst, CV_8UC3, cv::Scalar(128, 128, 128));
    int dx = (dst.width - nw) / 2, dy = (dst.height - nh) / 2;
    rs.copyTo(canvas(cv::Rect(dx, dy, nw, nh)));
    return {canvas, dx, dy};
}

void NnDetector::fitRec(std::vector<NnResult> &b, cv::Size2d ori, cv::Size2d now)
{
    double scale = std::max(ori.width / now.width, ori.height / now.height);
    for (auto &bb : b)
    {
        bb.box.x = (bb.box.x - now.width / 2) * scale + ori.width / 2;
        bb.box.y = (bb.box.y - now.height / 2) * scale + ori.height / 2;
        bb.box.width *= scale;
        bb.box.height *= scale;
        bb.center.x = (bb.center.x - now.width / 2) * scale + ori.width / 2;
        bb.center.y = (bb.center.y - now.height / 2) * scale + ori.height / 2;
    }
}