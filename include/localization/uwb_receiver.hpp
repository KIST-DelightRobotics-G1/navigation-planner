#pragma once

// UwbReceiver — our own subscriber for UWB position fixes (rt/kist/uwb/pose, geometry_msgs/
// PoseStamped: the tag publishes trilaterated x/y/z in the anchor frame, NO heading). Same shape
// as GoalReceiver (PoseStamped -> struct), so no ext-sensor-io link and no custom IDL. Used only
// for the localization seed: the UWB offset between the recorded map origin and the live boot
// pose gives T_map_odom's translation (yaw comes from matching the boot heading).

#include "common/data_buffer.hpp"

#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <string>

namespace unitree::robot { template <typename T> class ChannelSubscriber; }
namespace geometry_msgs::msg::dds_ { class PoseStamped_; }

namespace kist {

struct UwbFix {
    int64_t stamp_ns = 0;
    float   x = 0.f, y = 0.f;   // UWB anchor frame (m); no heading
};

class UwbReceiver {
public:
    UwbReceiver();
    ~UwbReceiver();
    UwbReceiver(const UwbReceiver&) = delete;
    UwbReceiver& operator=(const UwbReceiver&) = delete;

    bool start(int domain_id, const std::string& network_interface = "",
               const std::string& topic = "rt/kist/uwb/pose");
    void stop();

    DataBuffer<UwbFix> fix;   // latest UWB position (empty = no live fix)

private:
    void on_msg(const void* message);
    using Sub = unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PoseStamped_>;
    std::unique_ptr<Sub> sub_;
};

// Build an initial T_map_odom seed from UWB. Reads the recorded map-origin UWB from
// `sidecar_path` (a "x y" text file written by map_recorder), grabs the current fix (waiting up
// to wait_ms), and sets:
//   translation = R(map_uwb_yaw) * (uwb_now - uwb_map_origin)   (odom origin's position in map)
//   rotation    = R(seed_yaw)    (map<-odom heading; you match the boot heading, usually 0)
// Returns false with out_seed untouched if the sidecar is missing or no fix arrives — the caller
// then falls back to the config fixed seed.
bool uwb_compute_seed(UwbReceiver& uwb, const std::string& sidecar_path,
                      float map_uwb_yaw_rad, float seed_yaw_rad,
                      Eigen::Matrix4f& out_seed, int wait_ms = 2000);

} // namespace kist
