#include "cv/yolo/segmentation/yolo_seg_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace kist {

void yolo_seg_postprocess(const float* det, int det_count, int det_stride,
                          const float* proto, int proto_c, int proto_h, int proto_w,
                          const LetterboxTransform& lb, int orig_w, int orig_h,
                          float score_threshold, SegResult& out) {
    out.width  = orig_w;      out.height = orig_h;
    out.mask_width  = proto_w; out.mask_height = proto_h;
    out.input_width = lb.input_w; out.input_height = lb.input_h;
    out.scale = lb.scale; out.pad_x = lb.pad_x; out.pad_y = lb.pad_y;

    const int    num_masks  = proto_c;
    const size_t proto_area = static_cast<size_t>(proto_h) * proto_w;
    // protos as a [C, ph*pw] matrix for the coeff x proto GEMM.
    const cv::Mat proto_mat(num_masks, int(proto_area), CV_32F, const_cast<float*>(proto));
    const float sx = proto_w / float(lb.input_w), sy = proto_h / float(lb.input_h);
    const float scale = lb.scale;
    const int   px = lb.pad_x, py = lb.pad_y;

    // Pass 1: keep detections above threshold, gather their geometry + mask
    // coefficients into one [N, C] matrix. Boxes are in original px; pboxes are
    // the same boxes in proto coords (proto is aligned to the letterboxed input).
    struct Keep { cv::Rect box, pbox; float score; int class_id; };
    std::vector<Keep> keep;
    cv::Mat coeffs;   // [N, C], one row per kept detection

    for (int i = 0; i < det_count; ++i) {
        const float* d = det + size_t(i) * det_stride;
        const float score = d[4];
        if (score < score_threshold) continue;

        float x1 = (d[0] - px) / scale, y1 = (d[1] - py) / scale;
        float x2 = (d[2] - px) / scale, y2 = (d[3] - py) / scale;
        x1 = std::clamp(x1, 0.f, float(orig_w));  y1 = std::clamp(y1, 0.f, float(orig_h));
        x2 = std::clamp(x2, 0.f, float(orig_w));  y2 = std::clamp(y2, 0.f, float(orig_h));
        cv::Rect box(int(x1), int(y1), int(x2 - x1), int(y2 - y1));
        box &= cv::Rect(0, 0, orig_w, orig_h);
        if (box.width <= 0 || box.height <= 0) continue;

        cv::Rect pbox(int(std::floor(d[0] * sx)), int(std::floor(d[1] * sy)), 0, 0);
        pbox.width  = int(std::ceil(d[2] * sx)) - pbox.x;
        pbox.height = int(std::ceil(d[3] * sy)) - pbox.y;
        pbox &= cv::Rect(0, 0, proto_w, proto_h);
        if (pbox.width <= 0 || pbox.height <= 0) continue;

        coeffs.push_back(cv::Mat(1, num_masks, CV_32F, const_cast<float*>(d + 6)));
        keep.push_back({box, pbox, score, int(d[5])});
    }

    // Pass 2: one batched GEMM (coeffs [N,C] x protos [C,area] -> [N,area]),
    // then each mask is finished at proto resolution — crop to the box and
    // threshold the raw logit at 0 (sigmoid(x)>0.5 <=> x>0, so no exp needed).
    // No upsample, no full-frame alloc: cost is O(proto area), object-size
    // independent (Ultralytics process_mask, upsample=False).
    if (!keep.empty()) {
        const cv::Mat masks_all = coeffs * proto_mat;   // [N, ph*pw] logits
        for (size_t k = 0; k < keep.size(); ++k) {
            const cv::Mat m(proto_h, proto_w, CV_32F,
                            const_cast<float*>(masks_all.ptr<float>(int(k))));

            // crop_mask: keep the box region (proto coords), zero elsewhere.
            cv::Mat mask = cv::Mat::zeros(proto_h, proto_w, CV_8U);
            cv::Mat bin = m(keep[k].pbox) > 0.0f;   // logit>0 -> CV_8U 0/255
            bin.copyTo(mask(keep[k].pbox));

            SegDetection dobj;
            dobj.box      = keep[k].box;
            dobj.score    = keep[k].score;
            dobj.class_id = keep[k].class_id;
            dobj.mask     = std::move(mask);
            out.detections.push_back(std::move(dobj));
        }
    }
}

} // namespace kist
