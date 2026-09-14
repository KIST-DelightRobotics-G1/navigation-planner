#include "route_planner/perception/perception_worker.hpp"

#include <chrono>
#include <cstdint>

namespace kist {

void PerceptionWorker::start(LioReceiver& rx, LioTransformProducer& prod,
                             DataBuffer<ObstacleGrid>& grid_buf, DataBuffer<Costmap>& costmap_buf) {
    rx_ = &rx; prod_ = &prod; grid_buf_ = &grid_buf; costmap_buf_ = &costmap_buf;
    running_ = true;
    thread_ = std::thread(&PerceptionWorker::run, this);
}

void PerceptionWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void PerceptionWorker::run() {
    int64_t last_stamp = 0;
    while (running_) {
        auto scan = rx_->cloud_buf.GetData();
        if (!scan || scan->stamp_ns == last_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        last_stamp = scan->stamp_ns;
        auto rt = prod_->nearest(scan->stamp_ns);
        if (!rt) continue;                            // no pose for this scan
        mapper_.update_map(*scan, *rt);
        grid_buf_->SetData(mapper_.grid());
        costmap_buf_->SetData(mapper_.costmap());
    }
}

} // namespace kist
