#pragma once

// DDS Rx for the G1 lowstate (rt/lowstate) -> the FULL UnitreeState (29 joints + pelvis
// IMU + tick/mode), CRC-validated. The reader stays a pure transport layer: it hands out
// the whole state and consumers parse what they need — waist joints (12/13/14) for the
// waist FK, leg joints for foot FK, dq for safety, etc. Same shape as
// kist-gearsonic-inference's UnitreeStateReader.
//
// Singleton/subscriber like the other readers; needs the unitree ChannelFactory
// initialized first (any reader does that).

#include "common/data_buffer.hpp"
#include "unitree/unitree_state.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <string>

namespace kist {

class UnitreeStateReader {
public:
    static UnitreeStateReader& instance();
    bool start(int domain_id, const std::string& network_interface);
    void stop();

    // Latest decoded lowstate (read from any thread). Consumers parse the fields.
    DataBuffer<UnitreeState> state_buf;

private:
    UnitreeStateReader() = default;
    void on_lowstate(const void* message);

    using SdkLowState = unitree_hg::msg::dds_::LowState_;
    using LowStateSub = unitree::robot::ChannelSubscriber<SdkLowState>;
    unitree::robot::ChannelSubscriberPtr<SdkLowState> sub_;
};

} // namespace kist
