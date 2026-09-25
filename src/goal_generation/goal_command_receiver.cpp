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
            g.in_map  = true;   // catalog coords are map-frame -> converted to odom downstream
            // Per-destination terminal (dock) behavior — all optional, default OFF.
            g.dock.align      = d["align"].as<bool>(false);
            g.dock.approach   = d["approach"].as<bool>(false);
            g.dock.standoff_m = d["standoff_m"].as<float>(g.dock.standoff_m);
            g.dock.trigger_m  = d["trigger_m"].as<float>(g.dock.trigger_m);
            g.dock.speed      = d["speed"].as<float>(g.dock.speed);
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
        sub_->InitChannel([this](const void* m) { on_command(m); }, 10);   // RELIABLE, depth 10
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
    const auto& msg = *static_cast<const cortex_msgs::msg::dds_::SubtaskCmd_*>(message);
    const std::string plan_id = msg.plan_id();
    const uint16_t    index   = msg.index();
    const std::string action  = msg.action();

    auto emit = [&](GoalDisposition disp, const std::string& note) {
        Goal g;                                  // valid = false
        g.disp = disp; g.note = note;
        g.plan_id = plan_id; g.index = index; g.action = action;
        goal_buf_.SetData(g);
    };

    if (msg.cancel()) {                          // cancel (plan_id, index) — nav stops immediately
        emit(GoalDisposition::Cancelled, "cancelled");
        std::cout << "[GoalCommandReceiver] cancel (plan=" << plan_id << " idx=" << index << ")\n";
        return;
    }
    if (action != "move_to") {                   // nav only drives; other actions are VLA's
        emit(GoalDisposition::Failed, "unsupported: action '" + action + "' (nav handles move_to)");
        std::cerr << "[GoalCommandReceiver] unsupported action '" << action << "' — FAILED\n";
        return;
    }
    if (msg.args().size() < 1 || msg.args()[0].empty()) {
        emit(GoalDisposition::Failed, "unsupported: move_to needs a destination arg");
        std::cerr << "[GoalCommandReceiver] move_to with no destination arg — FAILED\n";
        return;
    }
    const std::string name = msg.args()[0];
    const auto it = catalog_.find(name);
    if (it == catalog_.end()) {
        emit(GoalDisposition::Failed, "unsupported: unknown destination '" + name + "'");
        std::cerr << "[GoalCommandReceiver] unknown destination '" << name << "' — FAILED\n";
        return;
    }
    Goal g = it->second;                         // catalog template: valid=true, xy/yaw/dock
    g.plan_id = plan_id; g.index = index; g.action = action;
    g.disp = GoalDisposition::None; g.note.clear();
    goal_buf_.SetData(g);
    std::cout << "[GoalCommandReceiver] move_to '" << name << "' (plan=" << plan_id
              << " idx=" << index << ") -> (" << g.x << ", " << g.y << ")\n";
}

} // namespace kist
