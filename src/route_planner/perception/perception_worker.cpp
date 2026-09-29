#include "route_planner/perception/perception_worker.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>

namespace kist {

void PerceptionWorker::start(LioReceiver& rx, LioTransformProducer& prod,
                             DataBuffer<ObstacleGrid>& grid_buf, DataBuffer<Costmap>& costmap_buf,
                             DataBuffer<Transform>& leveled_buf) {
    rx_ = &rx; prod_ = &prod; grid_buf_ = &grid_buf; costmap_buf_ = &costmap_buf;
    leveled_buf_ = &leveled_buf;
    running_ = true;
    thread_ = std::thread(&PerceptionWorker::run, this);
}

void PerceptionWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void PerceptionWorker::run() {
    int64_t last_stamp = 0;
    int     last_acc   = -1;
    while (running_) {
        auto scan = rx_->cloud_buf.GetData();
        if (!scan || scan->stamp_ns == last_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        last_stamp = scan->stamp_ns;
        auto rt = prod_->nearest(scan->stamp_ns);
        if (!rt) continue;                            // no pose for this scan

        // Ground-leveling estimate (DIAGNOSTIC ONLY for now — grid still built in raw odom below).
        const bool just_locked = leveler_.update(*scan, rt->T_odom_lidar, rt->T_odom_pelvis);
        const int  acc = leveler_.accepted_frames();
        if (just_locked)
            std::printf("[ground_leveler] LOCKED after %d fits — floor tilt = %.2f deg\n",
                        acc, leveler_.tilt_deg());
        else if (!leveler_.locked() && acc != last_acc)
            std::printf("[ground_leveler] fit %d/%d  tilt = %.2f deg\n",
                        acc, leveler_.config().lock_frames, leveler_.tilt_deg());
        last_acc = acc;
        leveled_buf_->SetData(leveler_.T_leveled_odom());   // publish for viz + downstream

        mapper_.update_map(*scan, *rt);               // unchanged: grid in raw odom (Phase 1)
        grid_buf_->SetData(mapper_.grid());
        costmap_buf_->SetData(mapper_.costmap());
    }
}

} // namespace kist
