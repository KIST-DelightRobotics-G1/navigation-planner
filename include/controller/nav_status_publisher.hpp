#pragma once

// NavStatusPublisher — publishes the controller's navigation state as a kist_msgs::NavStatus on
// rt/kist/nav/status, the back-channel that closes the run-to-completion loop: a peer / LLM that
// sent a named goal watches this to learn when the robot has DRIVING -> ... -> ARRIVED (or BLOCKED).
// Mirrors NavCommandPublisher (out-of-line ctor/dtor so the header needs no generated type / SDK).
//
// The DEBOUNCE (only emit ARRIVED after a stable hold) lives in the ControllerWorker; this class
// just puts a state code on the wire.

#include "controller/nav_command.hpp"   // kist::NavState

#include <memory>
#include <string>

namespace unitree::robot { template <typename T> class ChannelPublisher; }
namespace kist_msgs { class NavStatus; }

namespace kist {

inline constexpr const char* kNavStatusTopic = "rt/kist/nav/status";

class NavStatusPublisher {
public:
    NavStatusPublisher();          // out-of-line (unique_ptr to incomplete ChannelPublisher)
    ~NavStatusPublisher();

    bool start(int domain_id, const std::string& network_interface = "",
               const std::string& topic = kNavStatusTopic);
    void publish(const std::string& name, NavState state);   // name = active goal ("" = idle)
    void stop();

    NavStatusPublisher(const NavStatusPublisher&) = delete;
    NavStatusPublisher& operator=(const NavStatusPublisher&) = delete;

private:
    using StatusPub = unitree::robot::ChannelPublisher<kist_msgs::NavStatus>;
    std::unique_ptr<StatusPub> pub_;
    unsigned long long         seq_ = 0;
};

} // namespace kist
