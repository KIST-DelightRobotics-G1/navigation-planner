// Live depth+semantic fusion viewer, in the ROBOT frame (camera extrinsics
// applied), for tuning the camera mount pose live.
//   ./test_fusion_viewer [config_path]        (default config/config.yaml)
// ONE RealsenseReceiver feeds both paths:
//   color -> YoloSemPipeline -> dense class map (SemSegFrame)
//   depth -> DepthFrame (color intrinsics, aligned to color)
// A LabeledCloudGenerator worker thread does the fusion (deproject depth + label
// + transform to the robot frame); main only retunes the mount, reads the latest
// LabeledCloud off its buffer, and renders. Two panels, each point colored by
// its semantic class:
//   TOP  — top-down X-Y (obstacle layout): +X forward up, +Y left left
//   SIDE — elevation X-Z (tilt tuning):    +X forward right, +Z up up
// Tune the mount live and copy the printed values into config/fusion:
//   rotation:  w/s pitch(down/up)  a/d yaw(left/right)  q/e roll(ccw/cw)
//   position:  r/f height(up/down) z/c lateral(left/right) t/g forward(fwd/back)
//              x/SPACE = height filter   0 = reset to config   ESC = quit
// Tuning goal: in SIDE, make the floor lie flat ALONG the white z=0 line.
// With a DISPLAY it opens a window; headless it writes /tmp/fusion_view.png.

#include "labeled_cloud/labeled_cloud_generator.hpp"
#include "labeled_cloud/labeled_cloud.hpp"
#include "labeled_cloud/camera_extrinsics.hpp"
#include "cv/yolo/yolo_pipeline.hpp"
#include "cv/yolo/semantic_segmentation/yolo_sem_seg_engine.hpp"
#include "system/realsense_receiver.hpp"   // embedded from kist-ext-sensor-io
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

namespace {
constexpr int    kPanelW  = 600;
constexpr int    kTopH    = 600;   // top-down panel height
constexpr int    kSideH   = 320;   // side panel height
constexpr int    kStride  = 2;     // subsample depth
constexpr double kTopPpm  = 70.0;  // px/m, top-down
constexpr double kSidePpm = 70.0;  // px/m, side
constexpr double kSideZTop = 3.2, kSideZBot = -1.2;   // side Z span (m)
const cv::Scalar kBg(30,30,30), kGrid(60,60,60), kAxis(95,95,95),
                 kGround(220,220,220), kBand(60,200,220), kText(225,225,225);

// Live-tunable mount pose + filter (shared by render).
struct View {
    float x=0, y=0, height=0.8f, pitch=45.f, yaw=0, roll=0;
    bool  filter=false; float min_z=0.2f, max_z=1.8f;
};

// Same per-id palette as test_sem_camera (BEV color == 2D overlay color).
const cv::Mat& palette() {
    static const cv::Mat lut = [] {
        cv::Mat p(256,1,CV_8UC3);
        for (int i=0;i<256;++i){
            cv::Mat hsv(1,1,CV_8UC3,cv::Scalar((i*47)%180,200,255)), bgr;
            cv::cvtColor(hsv,bgr,cv::COLOR_HSV2BGR);
            p.at<cv::Vec3b>(i,0)=bgr.at<cv::Vec3b>(0,0);
        }
        p.at<cv::Vec3b>(0,0)=cv::Vec3b(0,0,0);
        p.at<cv::Vec3b>(kNoClass,0)=cv::Vec3b(90,90,90);
        return p;
    }();
    return lut;
}

inline bool kept(const View& v, float z){ return !v.filter || (z>=v.min_z && z<=v.max_z); }

// Top-down X-Y, robot at bottom-center: +X(forward) up, +Y(left) left.
cv::Mat render_top(const LabeledCloud& c, const View& v) {
    cv::Mat img(kTopH,kPanelW,CV_8UC3,kBg);
    const int ox=kPanelW/2, oy=kTopH-1;
    for (int r=2;r*kTopPpm<kTopH;r+=2)
        cv::circle(img,{ox,oy},int(r*kTopPpm),kGrid,1,cv::LINE_AA);
    cv::line(img,{ox,0},{ox,kTopH},kAxis,1);
    const cv::Mat& lut=palette();
    for (size_t i=0;i<c.size();++i){
        const float X=c.xyz[3*i], Y=c.xyz[3*i+1], Z=c.xyz[3*i+2];
        if (!kept(v,Z)) continue;
        const int col=ox-int(std::lround(Y*kTopPpm)), row=oy-int(std::lround(X*kTopPpm));
        if (col<0||col>=kPanelW||row<0||row>=kTopH) continue;
        img.at<cv::Vec3b>(row,col)=lut.at<cv::Vec3b>(c.label[i],0);
    }
    cv::circle(img,{ox,oy},4,cv::Scalar(0,0,255),-1);
    cv::putText(img,"TOP  +X up  +Y left  (rings 2m)",{8,20},
                cv::FONT_HERSHEY_SIMPLEX,0.42,kText,1,cv::LINE_AA);
    return img;
}

// Side elevation X-Z, robot origin at left: +X(forward) right, +Z(up) up.
cv::Mat render_side(const LabeledCloud& c, const View& v) {
    cv::Mat img(kSideH,kPanelW,CV_8UC3,kBg);
    const int ox=40;
    auto zrow=[&](float z){ return int((kSideZTop - z)*kSidePpm)+5; };
    const int z0=zrow(0.f);
    cv::line(img,{0,z0},{kPanelW,z0},kGround,1);                 // ground z=0
    cv::line(img,{ox,0},{ox,kSideH},kAxis,1);                    // robot vertical
    if (v.filter) {                                             // height band
        cv::line(img,{0,zrow(v.min_z)},{kPanelW,zrow(v.min_z)},kBand,1);
        cv::line(img,{0,zrow(v.max_z)},{kPanelW,zrow(v.max_z)},kBand,1);
    }
    for (int r=2;ox+int(r*kSidePpm)<kPanelW;r+=2)               // range ticks
        cv::line(img,{ox+int(r*kSidePpm),z0-4},{ox+int(r*kSidePpm),z0+4},kAxis,1);
    const cv::Mat& lut=palette();
    for (size_t i=0;i<c.size();++i){
        const float X=c.xyz[3*i], Z=c.xyz[3*i+2];
        if (!kept(v,Z)) continue;
        const int col=ox+int(std::lround(X*kSidePpm)), row=zrow(Z);
        if (col<0||col>=kPanelW||row<0||row>=kSideH) continue;
        img.at<cv::Vec3b>(row,col)=lut.at<cv::Vec3b>(c.label[i],0);
    }
    cv::circle(img,{ox,z0},4,cv::Scalar(0,0,255),-1);
    cv::putText(img,"SIDE  +X right  +Z up  (white=floor z0)",{8,20},
                cv::FONT_HERSHEY_SIMPLEX,0.42,kText,1,cv::LINE_AA);
    return img;
}

cv::Mat render(const LabeledCloud& c, const View& v, int pts, int fps) {
    cv::Mat top=render_top(c,v), side=render_side(c,v), out;
    cv::vconcat(std::vector<cv::Mat>{top,side},out);
    char s[160];
    std::snprintf(s,sizeof s,"h=%.2f pitch=%.1f  filter %s [%.1f-%.1f]  %d pts  %d fps",
                  v.height,v.pitch, v.filter?"ON":"off", v.min_z,v.max_z, pts,fps);
    cv::putText(out,s,{8,kTopH+kSideH-10},cv::FONT_HERSHEY_SIMPLEX,0.45,kText,1,cv::LINE_AA);
    return out;
}
}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();

    const int domain_id = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return 1;

    YoloSemSegConfig cfg;
    double      target_fps = 30.0;
    std::string cam_name   = "head";
    if (const auto cv = root["cv_inference"]) {
        cfg.onnx_path = cv["semantic_onnx"].as<std::string>(cfg.onnx_path);
        target_fps    = cv["target_fps"].as<double>(target_fps);
        cam_name      = cv["camera"].as<std::string>(cam_name);
    }

    View v0;   // config defaults (also the 'r' reset target)
    if (const auto f = root["fusion"]) {
        if (const auto e = f["camera_extrinsics"]) {
            v0.x = e["x"].as<float>(v0.x);       v0.y = e["y"].as<float>(v0.y);
            v0.height = e["height"].as<float>(v0.height);
            v0.pitch  = e["pitch_deg"].as<float>(v0.pitch);
            v0.yaw    = e["yaw_deg"].as<float>(v0.yaw);
            v0.roll   = e["roll_deg"].as<float>(v0.roll);
        }
        if (const auto h = f["height_filter"]) {
            v0.filter = h["enabled"].as<bool>(v0.filter);
            v0.min_z  = h["min_z"].as<float>(v0.min_z);
            v0.max_z  = h["max_z"].as<float>(v0.max_z);
        }
    }
    View v = v0;

    RealsenseReceiver rx;
    if (!rx.start(domain_id, "", cam_name)) return 1;   // empty iface — NIC from the DDS xml

    YoloSemPipeline pipe;
    if (!pipe.start(cfg, target_fps, [&](cv::Mat& bgr, int64_t& stamp) -> bool {
        auto cf = rx.color().GetData();
        if (!cf || cf->empty()) return false;
        stamp = cf->stamp_ns;
        cv::Mat(cf->height, cf->width, CV_8UC3,
                const_cast<uint8_t*>(cf->data.data()), cf->stride_bytes).copyTo(bgr);
        return true;
    })) return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    const bool has_disp = [] { const char* e = std::getenv("DISPLAY"); return e && e[0]; }();
    std::printf("[test_fusion_viewer] domain=%d cam=%s - %s\n", domain_id, cam_name.c_str(),
                has_disp ? "window (w/s pitch  a/d yaw  q/e roll  r/f height  "
                           "z/c lateral  t/g forward  x filter  0 reset  ESC quit)"
                         : "headless -> /tmp/fusion_view.png");

    // Fusion runs on its own worker thread; main only tunes, reads, renders.
    LabeledCloudConfig gcfg;
    gcfg.stride     = kStride;
    gcfg.extrinsics = make_camera_extrinsics(v.x, v.y, v.height, v.pitch, v.yaw, v.roll);
    LabeledCloudGenerator gen;
    if (!gen.start(rx.depth(), pipe.result(), gcfg)) return 1;

    const LabeledCloud empty;
    uint64_t last_processed = 0;
    int      fps = 0;
    auto     window = std::chrono::steady_clock::now();

    while (!g_stop) {
        // Live retune (cheap; the worker picks it up on its next cycle).
        gen.set_extrinsics(make_camera_extrinsics(v.x, v.y, v.height, v.pitch, v.yaw, v.roll));
        auto cloudp = gen.result().GetData();
        const LabeledCloud& rc = cloudp ? *cloudp : empty;

        if (has_disp) {
            cv::imshow("depth+semantic fusion (robot frame)",
                       render(rc, v, int(rc.size()), fps));
            const int k = cv::waitKey(30);
            if (k == 27) break;                              // ESC quit
            bool ch = true;
            switch (k) {
                case 'w': v.pitch += 1.f;  break;            // tilt down more
                case 's': v.pitch -= 1.f;  break;            // tilt up
                case 'a': v.yaw   += 1.f;  break;            // pan left  (+Z rot)
                case 'd': v.yaw   -= 1.f;  break;            // pan right
                case 'q': v.roll  += 1.f;  break;            // roll ccw
                case 'e': v.roll  -= 1.f;  break;            // roll cw
                case 'r': v.height += 0.02f; break;          // up
                case 'f': v.height -= 0.02f; break;          // down
                case 'z': v.y += 0.02f; break;               // lateral left (+Y)
                case 'c': v.y -= 0.02f; break;               // lateral right
                case 't': v.x += 0.02f; break;               // forward
                case 'g': v.x -= 0.02f; break;               // back
                case ' ': case 'x': v.filter = !v.filter; break;  // some highgui backends eat SPACE
                case '0': v = v0; break;                     // reset to config
                default:  ch = false;
            }
            if (ch)
                std::printf("  x=%.2f y=%.2f h=%.2f  pitch=%.1f yaw=%.1f roll=%.1f  filter=%s\n",
                            v.x, v.y, v.height, v.pitch, v.yaw, v.roll, v.filter?"on":"off");
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            const uint64_t processed = gen.frames_processed();
            fps = int(processed - last_processed);
            last_processed = processed;
            std::printf("  %zu pts  %d fps   (h=%.2f pitch=%.1f filter=%s)\n",
                        rc.size(), fps, v.height, v.pitch, v.filter?"on":"off");
            if (!has_disp)
                cv::imwrite("/tmp/fusion_view.png", render(rc, v, int(rc.size()), fps));
        }
    }

    gen.stop();
    pipe.stop();
    rx.stop();
    return 0;
}
