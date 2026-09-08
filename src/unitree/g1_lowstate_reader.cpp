#include "unitree/g1_lowstate_reader.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <chrono>
#include <iostream>

namespace kist {

namespace { const std::string kLowStateTopic = "rt/lowstate"; }

G1LowStateReader& G1LowStateReader::instance() {
    static G1LowStateReader inst;
    return inst;
}

bool G1LowStateReader::start(int domain_id, const std::string& network_interface) {
    try {
        // No-op if the embedding process already initialized the factory.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        sub_.reset(new LowStateSub(kLowStateTopic));
        sub_->InitChannel([this](const void* m) { on_lowstate(m); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[G1LowStateReader] subscribe failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    std::cout << "[G1LowStateReader] listening on " << kLowStateTopic << "\n";
    return true;
}

void G1LowStateReader::stop() {
    sub_.reset();
}

void G1LowStateReader::on_lowstate(const void* message) {
    const auto& ls = *static_cast<const SdkLowState*>(message);
    const auto& m  = ls.motor_state();   // fixed array; G1 has 29 joints

    WaistJoints w;
    w.yaw   = float(m[12].q());   // waist_yaw
    w.roll  = float(m[13].q());   // waist_roll
    w.pitch = float(m[14].q());   // waist_pitch
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    w.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    w.valid = true;
    waist_buf.SetData(w);
}

} // namespace kist
