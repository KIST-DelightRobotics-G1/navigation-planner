// Cloud <-> IMU TIMESTAMP / per-point TIME probe.
//
// Straight-line walking stretches the map and fast rotation doubles it. Both are
// classic DESKEW-OFF symptoms: if the per-point time we feed FAST-LIO is wrong
// (wrong UNIT or wrong REFERENCE), the IMU undistortion collapses to "all points at
// one instant" and the raw sensor-frame sweep gets stacked un-compensated.
//
// This probe reads the RAW utlidar cloud + IMU (no decode assumptions) and prints,
// per frame, everything needed to pin the convention:
//
//   1. per-point `time` field, RAW values (first/last/min/max/span, monotonic?, sign)
//      -> UNIT:      span ~9.3e7 => ns | ~9.3e4 => us | ~93 => ms | ~0.093 => SECONDS
//      -> REFERENCE: all >= 0 & rising => scan BEGIN | all <= 0 => scan END
//      Our code assumes ns from begin (curvature = time*1e-6, FAST-LIO reads /1000 s).
//      If the field is actually SECONDS, time*1e-6 -> ~0 -> deskew silently OFF.
//   2. cloud header.stamp vs newest IMU stamp -> clock offset + begin/end hint.
//   3. cloud rate (~10 Hz) and IMU rate (~200 Hz) sanity.
//
//   ./test_timestamp_probe [config.yaml]   (walk a few steps while it runs)

#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// newest IMU stamp seen (ns), for the cloud<->imu cross-check
static std::atomic<int64_t> g_imu_last_ns{0};
static std::atomic<long>    g_imu_count{0};
static std::atomic<int64_t> g_imu_first_ns{0};

template <class Stamp>
static int64_t hdr_ns(const Stamp& t) {
    return int64_t(t.sec()) * 1000000000LL + t.nanosec();
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    unitree::robot::ChannelFactory::Instance()->Init(domain, "");

    // ---- IMU: just track stamp cadence + newest stamp ----
    static unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_>
        imu_sub("rt/utlidar/imu_livox_mid360");
    imu_sub.InitChannel([](const void* m) {
        const auto& imu = *static_cast<const sensor_msgs::msg::dds_::Imu_*>(m);
        const int64_t s = hdr_ns(imu.header().stamp());
        g_imu_last_ns.store(s, std::memory_order_relaxed);
        if (g_imu_first_ns.load() == 0) g_imu_first_ns.store(s);
        g_imu_count.fetch_add(1, std::memory_order_relaxed);
    }, 50);

    // ---- cloud: dump the per-point time convention ----
    static std::atomic<int> g_frames{0};
    static unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>
        cloud_sub("rt/utlidar/cloud_livox_mid360");
    cloud_sub.InitChannel([](const void* m) {
        const auto& c = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(m);
        const int64_t recv_imu = g_imu_last_ns.load(std::memory_order_relaxed);

        // locate the `time` field (accept ANY datatype, we read it as its real width)
        uint32_t toff = 0; uint8_t tdt = 0; bool has_t = false;
        for (const auto& f : c.fields())
            if (f.name() == "time" || f.name() == "t" || f.name() == "timestamp" ||
                f.name() == "offset_time") { toff = f.offset(); tdt = f.datatype(); has_t = true; break; }

        const int64_t cs = hdr_ns(c.header().stamp());
        const std::size_t step = c.point_step();
        const std::size_t n = step ? c.data().size() / step : 0;
        const uint8_t* blob = c.data().data();

        const int fr = g_frames.fetch_add(1) ;
        std::printf("\n── frame #%d  stamp=%lld ns  frame_id=\"%s\"  pts=%zu  step=%zu\n",
                    fr, (long long)cs, c.header().frame_id().c_str(), n, step);

        // list fields once (frame 0) so we SEE datatype/offset of `time`
        if (fr == 0) {
            std::printf("   fields:");
            for (const auto& f : c.fields())
                std::printf(" %s(dt=%u,off=%u)", f.name().c_str(), f.datatype(), f.offset());
            std::printf("\n   (PointField dt: 2=uint8 3=int8 4=uint16 5=int16 6=uint32 7=int32? 7=float32? -> 7=FLOAT32,6=UINT32,4=UINT16)\n");
        }

        // cloud <-> imu clock cross-check
        if (recv_imu != 0) {
            const double dt_ms = (recv_imu - cs) / 1e6;
            std::printf("   newestIMU - cloudStamp = % .1f ms   "
                        "(~0 => stamp≈scan END | ~+93 => stamp≈scan BEGIN, before latency)\n", dt_ms);
        }

        if (!has_t || n == 0) { std::printf("   NO per-point time field!\n"); return; }

        // read time as BOTH float32 and uint32 so the true unit is unambiguous
        auto read_f = [&](std::size_t i) { float v; std::memcpy(&v, blob + i*step + toff, 4); return double(v); };
        auto read_u = [&](std::size_t i) { uint32_t v; std::memcpy(&v, blob + i*step + toff, 4); return double(v); };
        const bool as_float = (tdt == 7);

        double first = as_float ? read_f(0) : read_u(0);
        double last  = as_float ? read_f(n-1) : read_u(n-1);
        double tmin = first, tmax = first;
        int decreases = 0; double prev = first;
        for (std::size_t i = 0; i < n; ++i) {
            const double v = as_float ? read_f(i) : read_u(i);
            if (v < tmin) tmin = v; if (v > tmax) tmax = v;
            if (v < prev - 1e-12) ++decreases;
            prev = v;
        }
        const double span = tmax - tmin;
        std::printf("   time[%s]  first=%.6g last=%.6g  min=%.6g max=%.6g  span=%.6g  "
                    "decreasing_steps=%d/%zu\n",
                    as_float ? "float32" : "uint32", first, last, tmin, tmax, span, decreases, n);

        // interpret span as each candidate unit -> which one is ~one 10 Hz frame (0.1 s)?
        std::printf("   span as: %.3f s | %.3f ms | %.1f us | %.0f ns\n",
                    span, span*1e3, span*1e6, span*1e9);          // if span is SECONDS
        std::printf("   span as: %.6f s (if raw=ns) | %.6f s (if raw=us) | %.6f s (if raw=ms)\n",
                    span*1e-9, span*1e-6, span*1e-3);
        std::printf("   -> reference: %s\n",
                    (tmin >= -1e-9 ? "all >= 0  => BEGIN (points forward from stamp)"
                                   : "has < 0   => END (points backward from stamp)"));
        std::printf("   -> OUR pipeline feeds FAST-LIO curvature=time*1e-6 (ns->ms), reads /1000=s\n"
                    "      => effective per-point dt = raw*1e-9 s = %.6f s over the frame\n",
                    span*1e-9);
        std::printf("      (needs to be ~0.09 s to deskew a 10 Hz frame; if ~0 => DESKEW OFF)\n");
    }, 5);

    std::printf("timestamp probe running — WALK a few steps. Ctrl+C to stop.\n");
    for (int i = 0; i < 200 && g_frames.load() < 12; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // IMU rate
    const long ic = g_imu_count.load();
    const int64_t i0 = g_imu_first_ns.load(), i1 = g_imu_last_ns.load();
    if (ic > 1 && i1 > i0)
        std::printf("\nIMU: %ld samples over %.2f s -> %.1f Hz (expect ~200)\n",
                    ic, (i1 - i0)/1e9, ic / ((i1 - i0)/1e9));
    std::printf("done (%d cloud frames).\n", g_frames.load());
    return 0;
}
