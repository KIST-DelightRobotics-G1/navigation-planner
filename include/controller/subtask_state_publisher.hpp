#pragma once

// SubtaskStatePublisher — publishes the controller's subtask state as a cortex_msgs::msg::dds_::SubtaskState_ on
// rt/cortex/nav/state (ROS2 topic /cortex/nav/state), RELIABLE, at 10 Hz. This is the back-channel
// that closes the orchestrator's run-to-completion loop opened by SubtaskCmd on /cortex/nav/cmd.
// Passive (driven by the controller loop; no own thread); mirrors NavCommandPublisher's PIMPL so
// the header needs no generated type / SDK.
//
// The debounce / progress / status mapping lives in the ControllerWorker; this class just puts the
// fields on the wire (and stamps the header).

#include <cstdint>
#include <memory>
#include <string>

namespace unitree::robot { template <typename T> class ChannelPublisher; }
namespace kist_msgs { class SubtaskState; }

namespace kist {

inline constexpr const char* kNavStateTopic = "rt/cortex/nav/state";   // ROS2 /cortex/nav/state

// SubtaskState.status wire codes (shared contract with the orchestrator / VLA).
enum class SubtaskStatus : uint8_t { Idle = 0, Running = 1, Done = 2, Failed = 3 };

class SubtaskStatePublisher {
public:
    SubtaskStatePublisher();          // out-of-line (unique_ptr to incomplete ChannelPublisher)
    ~SubtaskStatePublisher();

    bool start(int domain_id, const std::string& network_interface = "",
               const std::string& topic = kNavStateTopic);
    // Publish one SubtaskState (header.stamp = now). plan_id/index/action echo the reported subtask
    // ("" / 0 / "" when idle); progress is 0..1; note is human-readable ("" when nothing to add).
    void publish(const std::string& plan_id, uint16_t index, const std::string& action,
                 SubtaskStatus status, float progress, const std::string& detail);
    void stop();

    SubtaskStatePublisher(const SubtaskStatePublisher&) = delete;
    SubtaskStatePublisher& operator=(const SubtaskStatePublisher&) = delete;

private:
    using Pub = unitree::robot::ChannelPublisher<cortex_msgs::msg::dds_::SubtaskState_>;
    std::unique_ptr<Pub> pub_;
};

} // namespace kist
