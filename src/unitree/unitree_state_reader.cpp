#include "unitree/unitree_state_reader.hpp"

#include "unitree/crc32.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <chrono>
#include <iostream>

namespace kist {

namespace {
const std::string kLowStateTopic = "rt/lowstate";

IMU to_imu(const unitree_hg::msg::dds_::IMUState_& s) {
    IMU o;
    o.quaternion    = {s.quaternion()[0],    s.quaternion()[1],    s.quaternion()[2], s.quaternion()[3]};
    o.gyroscope     = {s.gyroscope()[0],     s.gyroscope()[1],     s.gyroscope()[2]};
    o.accelerometer = {s.accelerometer()[0], s.accelerometer()[1], s.accelerometer()[2]};
    return o;
}

UnitreeState convert(const unitree_hg::msg::dds_::LowState_& src) {
    UnitreeState out;
    out.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch()).count();
    out.tick         = src.tick();
    out.mode_machine = src.mode_machine();
    out.imu_pelvis   = to_imu(src.imu_state());
    const auto& m = src.motor_state();
    for (int i = 0; i < kNumMotors; ++i) {
        out.motors[i].q   = m[i].q();
        out.motors[i].dq  = m[i].dq();
        out.motors[i].tau = m[i].tau_est();
    }
    return out;
}
} // namespace

UnitreeStateReader& UnitreeStateReader::instance() {
    static UnitreeStateReader inst;
    return inst;
}

bool UnitreeStateReader::start(int domain_id, const std::string& network_interface) {
    try {
        // No-op if the embedding process already initialized the factory.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        sub_.reset(new LowStateSub(kLowStateTopic));
        sub_->InitChannel([this](const void* m) { on_lowstate(m); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[UnitreeStateReader] subscribe failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    std::cout << "[UnitreeStateReader] listening on " << kLowStateTopic << "\n";
    return true;
}

void UnitreeStateReader::stop() { sub_.reset(); }

void UnitreeStateReader::on_lowstate(const void* message) {
    const auto& ls = *static_cast<const SdkLowState*>(message);

    // Drop corrupted packets — the state feeds FK / safety directly.
    const uint32_t crc = crc32_core(reinterpret_cast<const uint32_t*>(&ls),
                                    (sizeof(SdkLowState) >> 2) - 1);
    if (crc != ls.crc()) {
        std::cerr << "[UnitreeStateReader] LowState CRC mismatch — packet dropped\n";
        return;
    }
    state_buf.SetData(convert(ls));
}

} // namespace kist
