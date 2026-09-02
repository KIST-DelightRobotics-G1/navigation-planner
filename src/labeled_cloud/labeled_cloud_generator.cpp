#include "labeled_cloud/labeled_cloud_generator.hpp"

#include <pthread.h>

#include <chrono>

namespace kist {

bool LabeledCloudGenerator::start(DataBuffer<DepthFrame>&  depth_src,
                                  DataBuffer<SemSegFrame>& mask_src,
                                  const LabeledCloudConfig& cfg) {
    if (running_) return true;
    depth_src_ = &depth_src;
    mask_src_  = &mask_src;
    stride_    = cfg.stride > 0 ? cfg.stride : 1;
    { std::lock_guard<std::mutex> lk(cfg_mtx_); extrinsics_ = cfg.extrinsics; }
    running_ = true;
    thread_  = std::thread(&LabeledCloudGenerator::run, this);
    return true;
}

void LabeledCloudGenerator::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void LabeledCloudGenerator::run() {
    pthread_setname_np(pthread_self(), "labeled-cloud");
    int64_t      last_stamp = -1;
    LabeledCloud cam, robot;   // reused scratch across cycles

    while (running_) {
        auto d = depth_src_->GetData();
        // Follow the depth rate; latest-wins pairs it with the freshest mask.
        if (d && !d->empty() && d->stamp_ns != last_stamp) {
            last_stamp = d->stamp_ns;
            auto m = mask_src_->GetData();
            fuse_depth_semantic(*d, m ? *m : SemSegFrame{}, cam, stride_);

            CameraExtrinsics e;
            { std::lock_guard<std::mutex> lk(cfg_mtx_); e = extrinsics_; }
            transform_cloud(cam, e, robot);

            cloud_buf_.SetData(robot);
            processed_.fetch_add(1, std::memory_order_relaxed);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
