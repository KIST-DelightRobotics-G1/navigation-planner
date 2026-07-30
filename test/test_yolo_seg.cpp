// Static-image check for YoloSegEngine (no camera, no DDS) — the first proof
// that the vendored TensorRT backend + YOLO26-seg postprocess actually works.
//   ./test_yolo_seg <image> [onnx_path]     (onnx defaults to models/yolo26l-seg.onnx)
// Overlays instance masks + boxes + class/score and prints detections. With a
// DISPLAY it opens a window; headless it writes /tmp/yolo_seg_out.png.
//
// First run builds+caches the .trt engine (slow, ~1min); later runs load it.

#include "cv/yolo/segmentation/yolo_seg_engine.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace kist;

namespace {
const char* kCoco[80] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"};

cv::Scalar color_for(int id) {
    // deterministic bright-ish color per class
    int h = (id * 47) % 180;
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, 200, 255)), bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
    return cv::Scalar(c[0], c[1], c[2]);
}
const char* name_of(int id) { return (id >= 0 && id < 80) ? kCoco[id] : "?"; }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <image> [onnx_path]\n", argv[0]);
        return 2;
    }
    cv::Mat img = cv::imread(argv[1]);
    if (img.empty()) {
        std::fprintf(stderr, "[test_yolo_seg] cannot read image: %s\n", argv[1]);
        return 1;
    }

    YoloSegConfig cfg;
    if (argc >= 3) cfg.onnx_path = argv[2];

    std::printf("[test_yolo_seg] loading engine (%s) - first build may take ~1min\n",
                cfg.onnx_path.c_str());
    YoloSegEngine engine;
    if (!engine.init(cfg)) return 1;

    // Timed inference (a warmup pass first so the timing excludes lazy setup).
    engine.infer(img);
    const auto t0 = std::chrono::steady_clock::now();
    SegResult r = engine.infer(img, 1);
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("[test_yolo_seg] %zu detections in %.1f ms (%.1f fps)\n",
                r.detections.size(), ms, 1000.0 / ms);

    // Overlay.
    cv::Mat vis = img.clone();
    for (const auto& d : r.detections) {
        const cv::Scalar col = color_for(d.class_id);
        cv::Rect box = d.box & cv::Rect(0, 0, vis.cols, vis.rows);
        if (box.width > 0 && box.height > 0 && !d.mask.empty()) {
            // Masks are at proto resolution: map the box into mask space, then
            // upsample just that sub-region to the box for the overlay.
            const cv::Point2f m0 = r.orig_to_mask(box.x, box.y);
            const cv::Point2f m1 = r.orig_to_mask(box.x + box.width, box.y + box.height);
            cv::Rect mrect(cvFloor(m0.x), cvFloor(m0.y),
                           cvCeil(m1.x - m0.x), cvCeil(m1.y - m0.y));
            mrect &= cv::Rect(0, 0, d.mask.cols, d.mask.rows);
            if (mrect.width > 0 && mrect.height > 0) {
                cv::Mat mbox;
                cv::resize(d.mask(mrect), mbox, box.size(), 0, 0, cv::INTER_NEAREST);
                cv::Mat colored(box.size(), CV_8UC3, col);
                colored.copyTo(vis(box), mbox);
            }
        }
        cv::rectangle(vis, d.box, col, 2);
        char label[64];
        std::snprintf(label, sizeof label, "%s %.2f", name_of(d.class_id), d.score);
        cv::putText(vis, label, {d.box.x, std::max(0, d.box.y - 5)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv::LINE_AA);
        std::printf("  %-14s %.2f  box=[%d,%d %dx%d]\n",
                    name_of(d.class_id), d.score, d.box.x, d.box.y, d.box.width, d.box.height);
    }
    cv::addWeighted(vis, 0.6, img, 0.4, 0, vis);           // blend masks with the image

    const bool has_disp = [] { const char* e = std::getenv("DISPLAY"); return e && e[0]; }();
    if (has_disp) {
        cv::imshow("YOLO26-seg", vis);
        std::printf("[test_yolo_seg] window open - any key to quit\n");
        cv::waitKey(0);
    } else {
        cv::imwrite("/tmp/yolo_seg_out.png", vis);
        std::printf("[test_yolo_seg] headless -> /tmp/yolo_seg_out.png\n");
    }
    return 0;
}
