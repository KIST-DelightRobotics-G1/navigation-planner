#include "goal_generation/goal_command_receiver.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>

namespace kist {

GoalCommandReceiver::~GoalCommandReceiver() { stop(); }

bool GoalCommandReceiver::load(const std::string& path) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const auto ds = root["destinations"];
        if (!ds || !ds.IsSequence()) {
            std::cerr << "[GoalCommandReceiver] no 'destinations' list in " << path << "\n";
            return false;
        }
        catalog_.clear();
        for (const auto& d : ds) {
            Goal g;
            g.name    = d["name"].as<std::string>("");
            g.x       = d["x"].as<float>(0.0f);
            g.y       = d["y"].as<float>(0.0f);
            g.yaw     = d["yaw_deg"].as<float>(0.0f) * float(M_PI) / 180.0f;   // deg -> rad
            g.has_yaw = true;
            g.valid   = true;
            if (!g.name.empty()) catalog_[g.name] = g;
        }
    } catch (const std::exception& e) {
        std::cerr << "[GoalCommandReceiver] load failed (" << path << "): " << e.what() << "\n";
        return false;
    }
    return true;
}

bool GoalCommandReceiver::start(int domain_id, const std::string& network_interface,
                                const std::string& destinations_yaml, const std::string& topic) {
    if (sub_) return true;
    if (!load(destinations_yaml)) return false;
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        sub_ = std::make_shared<Sub>(topic);
        sub_->InitChannel([this](const void* m) { on_command(m); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[GoalCommandReceiver] DDS subscribe failed on \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    std::cout << "[GoalCommandReceiver] " << catalog_.size()
              << " destinations, listening on " << topic << "\n";
    return true;
}

void GoalCommandReceiver::stop() {
    sub_.reset();
}

void GoalCommandReceiver::on_command(const void* message) {
    const auto& msg = *static_cast<const kist_msgs::NavGoalCommand*>(message);
    const std::string name = msg.name();

    if (name.empty()) {                          // cancel / stop
        goal_buf_.SetData(Goal{});               // valid = false
        std::cout << "[GoalCommandReceiver] cancel\n";
        return;
    }
    const auto it = catalog_.find(name);
    if (it == catalog_.end()) {
        std::cerr << "[GoalCommandReceiver] unknown destination '" << name << "' — ignored\n";
        goal_buf_.SetData(Goal{});               // valid = false (no such goal)
        return;
    }
    goal_buf_.SetData(it->second);
    std::cout << "[GoalCommandReceiver] goal -> '" << name << "' ("
              << it->second.x << ", " << it->second.y << ")\n";
}

} // namespace kist
