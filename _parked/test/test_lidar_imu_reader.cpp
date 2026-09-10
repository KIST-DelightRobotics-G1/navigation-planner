// UnitreePointCloudImuReader test — drains the Mid-360 IMU hand-off queue on a
// consumer thread and reports rate + drops + the latest sample. Confirms the
// g->m/s^2 conversion (a stationary IMU's accel magnitude should read ~9.8, NOT
// ~1.0) and that no samples are lost through the queue.
//   ./test_lidar_imu_reader [config.yaml]

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "unitree/unitree_pointcloud_imu_reader.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <deque>
#include <mutex>
#include <thread>

using kist::ImuSample;
using kist::UnitreePointCloudImuReader;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    std::signal(SIGINT, [](int) { g_stop = true; });

    auto& rd = UnitreePointCloudImuReader::instance();
    if (!rd.start(domain, "")) return 1;

    std::atomic<uint64_t> total{0};
    ImuSample last{};
    std::mutex last_mtx;
    std::thread consumer([&] {
        std::deque<ImuSample> batch;
        while (rd.imu_queue.pop_all(batch)) {          // false only once closed+drained
            total.fetch_add(batch.size(), std::memory_order_relaxed);
            std::lock_guard<std::mutex> lk(last_mtx);
            last = batch.back();
        }
    });

    std::printf("draining rt/utlidar/imu_livox_mid360 (Ctrl+C to quit)...\n");
    uint64_t prev = 0;
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const uint64_t n = total.load();
        ImuSample s;
        { std::lock_guard<std::mutex> lk(last_mtx); s = last; }
        std::printf("imu: %llu total  ~%llu Hz  dropped=%llu | "
                    "gyro=(% .3f % .3f % .3f) accel=(% .2f % .2f % .2f) m/s^2 |accel|=%.2f\n",
                    static_cast<unsigned long long>(n),
                    static_cast<unsigned long long>(n - prev),
                    static_cast<unsigned long long>(rd.dropped.load()),
                    s.gyro.x(), s.gyro.y(), s.gyro.z(),
                    s.accel.x(), s.accel.y(), s.accel.z(), s.accel.norm());
        prev = n;
    }

    rd.stop();          // closes the queue -> consumer's pop_all returns false
    consumer.join();
    std::printf("total IMU samples: %llu, dropped: %llu\n",
                static_cast<unsigned long long>(total.load()),
                static_cast<unsigned long long>(rd.dropped.load()));
    return 0;
}
