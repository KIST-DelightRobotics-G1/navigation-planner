#pragma once

// Loads the destination catalog (config/destinations.yaml) and publishes it over
// DDS as a NavDestinationList, so any peer shares the same set of goals it can
// address by name. Published immediately on start, then re-published at a low
// rate so a late-joining subscriber still receives it (DDS volatile QoS drops a
// one-shot sample for anyone that joins after it).

#include <kist_nav.hpp>   // generated: kist_msgs::NavDestinationList

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot { template <typename T> class ChannelPublisher; }

namespace kist {

inline constexpr const char* kNavDestinationsTopic = "rt/kist/nav/destinations";

class DestinationPublisher {
public:
    DestinationPublisher();
    ~DestinationPublisher();   // out-of-line: unique_ptr<ChannelPublisher> needs the complete type
    DestinationPublisher(const DestinationPublisher&) = delete;
    DestinationPublisher& operator=(const DestinationPublisher&) = delete;

    // Load `destinations_yaml`, open the DDS channel, and start publishing the
    // catalog (immediately + every 1/republish_hz s). False on load / DDS failure.
    bool start(int domain_id, const std::string& network_interface,
               const std::string& destinations_yaml,
               const std::string& topic = kNavDestinationsTopic,
               double republish_hz = 1.0);
    void stop();

    size_t count() const { return count_; }   // destinations loaded
    bool   running() const { return running_; }

private:
    void run(double hz);
    bool load(const std::string& yaml_path);

    using Pub = unitree::robot::ChannelPublisher<kist_msgs::NavDestinationList>;
    std::unique_ptr<Pub>          pub_;
    kist_msgs::NavDestinationList msg_;    // prebuilt catalog (only seq/stamp change)
    size_t                        count_ = 0;
    std::thread                   thread_;
    std::atomic<bool>             running_{false};
};

} // namespace kist
