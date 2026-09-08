#pragma once

// DDS Rx for the G1 lowstate (rt/lowstate) — extracts the WAIST joint angles,
// the joints that tilt/translate the torso (and its head-mounted LiDAR + camera)
// relative to the pelvis as the robot walks. The pelvis pose (odom) misses this
// motion, so it feeds forward kinematics to compensate the sensor bob.
//
// Same singleton/subscriber shape as the other readers. Requires the unitree
// ChannelFactory to be initialized first (the pointcloud/odom readers do that).

#include "common/data_buffer.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <cstdint>
#include <string>

namespace kist {

// Waist joint angles (rad). Indices 12/13/14 in MuJoCo/DDS order.
struct WaistJoints {
    int64_t stamp_ns = 0;                       // receipt time (for later time-sync)
    float   yaw = 0.f, roll = 0.f, pitch = 0.f; // waist_yaw(12), waist_roll(13), waist_pitch(14)
    bool    valid = false;
};

class G1LowStateReader {
public:
    static G1LowStateReader& instance();
    bool start(int domain_id, const std::string& network_interface);
    void stop();

    DataBuffer<WaistJoints> waist_buf;

private:
    G1LowStateReader() = default;
    void on_lowstate(const void* message);

    using SdkLowState = unitree_hg::msg::dds_::LowState_;
    using LowStateSub = unitree::robot::ChannelSubscriber<SdkLowState>;
    unitree::robot::ChannelSubscriberPtr<SdkLowState> sub_;
};

} // namespace kist
