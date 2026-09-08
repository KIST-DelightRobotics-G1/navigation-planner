#include "goal_generation/destination_publisher.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <yaml-cpp/yaml.h>

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <iostream>

namespace kist {

DestinationPublisher::DestinationPublisher() = default;
DestinationPublisher::~DestinationPublisher() { stop(); }

bool DestinationPublisher::load(const std::string& path) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const auto ds = root["destinations"];
        if (!ds || !ds.IsSequence()) {
            std::cerr << "[DestinationPublisher] no 'destinations' list in " << path << "\n";
            return false;
        }
        msg_.destinations().clear();
        for (const auto& d : ds) {
            kist_msgs::NavDestination nd;
            nd.name(d["name"].as<std::string>(""));
            nd.x(d["x"].as<float>(0.0f));
            nd.y(d["y"].as<float>(0.0f));
            nd.yaw_deg(d["yaw_deg"].as<float>(0.0f));
            msg_.destinations().push_back(nd);
        }
        count_ = msg_.destinations().size();
    } catch (const std::exception& e) {
        std::cerr << "[DestinationPublisher] load failed (" << path << "): " << e.what() << "\n";
        return false;
    }
    return true;
}

bool DestinationPublisher::start(int domain_id, const std::string& network_interface,
                                 const std::string& destinations_yaml,
                                 const std::string& topic, double republish_hz) {
    if (running_) return true;
    if (!load(destinations_yaml) || count_ == 0) return false;
    try {
        // No-op if the embedding process already initialized the factory.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        pub_.reset(new Pub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[DestinationPublisher] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    std::cout << "[DestinationPublisher] publishing " << count_ << " destinations on " << topic << "\n";
    running_ = true;
    thread_  = std::thread(&DestinationPublisher::run, this, republish_hz);
    return true;
}

void DestinationPublisher::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    pub_.reset();
}

void DestinationPublisher::run(double hz) {
    pthread_setname_np(pthread_self(), "dest-pub");
    const auto period_ms = std::chrono::milliseconds(int(1000.0 / std::max(0.1, hz)));
    unsigned long long seq = 0;
    auto next = std::chrono::steady_clock::now();
    while (running_) {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        msg_.seq(seq++);
        msg_.stamp_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
        pub_->Write(msg_);
        // Sleep in small steps so stop() is responsive despite the slow cadence.
        next += period_ms;
        while (running_ && std::chrono::steady_clock::now() < next)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

} // namespace kist
