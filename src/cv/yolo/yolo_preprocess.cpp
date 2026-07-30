#include "cv/yolo/yolo_preprocess.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace kist {

void yolo_preprocess(const cv::Mat& bgr, int input_w, int input_h,
                     float* dst, LetterboxTransform& lb,
                     YoloPreprocessScratch& scratch) {
    const int   ow = bgr.cols, oh = bgr.rows;
    const float scale = std::min(input_w / float(ow), input_h / float(oh));
    const int   nw = int(std::round(ow * scale));
    const int   nh = int(std::round(oh * scale));
    const int   px = (input_w - nw) / 2;
    const int   py = (input_h - nh) / 2;

    lb.scale = scale;  lb.pad_x = px;  lb.pad_y = py;
    lb.input_w = input_w;  lb.input_h = input_h;

    cv::resize(bgr, scratch.resized, cv::Size(nw, nh));
    if (scratch.canvas.empty())
        scratch.canvas.create(input_h, input_w, CV_8UC3);
    scratch.canvas.setTo(cv::Scalar(114, 114, 114));
    scratch.resized.copyTo(scratch.canvas(cv::Rect(px, py, nw, nh)));

    // BGR->RGB, /255, HWC->CHW into the reused blob (empty Size => no internal
    // resize; canvas is already input-sized), then into the caller's buffer.
    cv::dnn::blobFromImage(scratch.canvas, scratch.blob, 1.0 / 255.0, cv::Size(),
                           cv::Scalar(), /*swapRB=*/true, /*crop=*/false, CV_32F);
    std::memcpy(dst, scratch.blob.ptr<float>(),
                size_t(3) * input_h * input_w * sizeof(float));
}

} // namespace kist
