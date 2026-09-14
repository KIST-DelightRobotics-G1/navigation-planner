#include "localization/localization_worker.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>

namespace kist {

bool LocalizationWorker::start(LioReceiver& rx, const std::string& prior_pcd,
                               const Eigen::Matrix4f& seed, DataBuffer<MapOdom>& out) {
    if (!reloc_.load_prior(prior_pcd)) {
        std::cerr << "[LocalizationWorker] prior map load failed: " << prior_pcd << "\n";
        return false;
    }
    reloc_.seed(seed);
    rx_ = &rx; out_ = &out;
    running_ = true;
    thread_ = std::thread(&LocalizationWorker::run, this);
    return true;
}

void LocalizationWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void LocalizationWorker::run() {
    int64_t last_stamp = 0;
    auto    last_align = std::chrono::steady_clock::now();
    auto    last_print = last_align;
    bool    locked = false;
    while (running_) {
        auto cloud = rx_->cloud_buf.GetData();
        if (cloud && cloud->stamp_ns != last_stamp) {
            last_stamp = cloud->stamp_ns;
            reloc_.add_scan(*cloud);
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_align >= std::chrono::milliseconds(1000)) {   // ~1 Hz align
            last_align = now;
            double fit = -1.0;
            if (reloc_.align(&fit)) {                                // accepted -> publish map->odom
                MapOdom mo; mo.T_map_odom = reloc_.T_map_odom();
                out_->SetData(mo);
                locked = true;
            }
            if (now - last_print >= std::chrono::milliseconds(2000)) {
                last_print = now;
                std::printf("[localization] %s submap=%zu fitness=%.4f\n",
                            locked ? "LOCK" : "search", reloc_.submap_size(), fit);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace kist
