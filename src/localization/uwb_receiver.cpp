#include "localization/uwb_receiver.hpp"

#include <unitree/idl/ros2/PoseStamped_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

namespace kist {

UwbReceiver::UwbReceiver() = default;
UwbReceiver::~UwbReceiver() { stop(); }

bool UwbReceiver::start(int domain_id, const std::string& network_interface,
                        const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);  // no-op if inited
        sub_.reset(new Sub(topic));
        sub_->InitChannel([this](const void* m) { on_msg(m); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[UwbReceiver] DDS init failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[UwbReceiver] listening on " << topic << "\n";
    return true;
}

void UwbReceiver::stop() { sub_.reset(); }

void UwbReceiver::on_msg(const void* message) {
    const auto& msg = *static_cast<const geometry_msgs::msg::dds_::PoseStamped_*>(message);
    const auto& p = msg.pose().position();
    UwbFix f;
    f.stamp_ns = int64_t(msg.header().stamp().sec()) * 1000000000LL + msg.header().stamp().nanosec();
    f.x = float(p.x());
    f.y = float(p.y());
    fix.SetData(f);
}

bool uwb_compute_seed(UwbReceiver& uwb, const std::string& sidecar_path,
                      float map_uwb_yaw_rad, float seed_yaw_rad,
                      Eigen::Matrix4f& out_seed, int wait_ms) {
    // recorded map-origin UWB (x y) from the sidecar
    std::ifstream f(sidecar_path);
    float ox = 0.f, oy = 0.f;
    if (!f || !(f >> ox >> oy)) {
        std::cerr << "[uwb_seed] no sidecar at " << sidecar_path << "\n";
        return false;
    }
    // current fix (wait up to wait_ms)
    UwbFix now;
    bool have = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (auto fx = uwb.fix.GetData()) { now = *fx; have = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!have) { std::cerr << "[uwb_seed] no live UWB fix within " << wait_ms << " ms\n"; return false; }

    // translation = R(map_uwb_yaw) * (uwb_now - uwb_map_origin);  rotation = R(seed_yaw)
    const float dx = now.x - ox, dy = now.y - oy;
    const float c = std::cos(map_uwb_yaw_rad), s = std::sin(map_uwb_yaw_rad);
    out_seed = Eigen::Matrix4f::Identity();
    out_seed.block<3,3>(0,0) = Eigen::AngleAxisf(seed_yaw_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    out_seed(0,3) = c*dx - s*dy;
    out_seed(1,3) = s*dx + c*dy;
    std::cout << "[uwb_seed] map_origin=(" << ox << "," << oy << ") now=(" << now.x << "," << now.y
              << ") -> seed t=(" << out_seed(0,3) << "," << out_seed(1,3) << ")\n";
    return true;
}

} // namespace kist
