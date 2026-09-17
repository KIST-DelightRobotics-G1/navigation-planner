#include "system/map_recorder.hpp"

#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <csignal>
#include <iostream>

namespace kist {

namespace { MapRecorderSystem* g_self = nullptr; }

bool MapRecorderSystem::start(const std::string& config_path) {
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return false;

    if (!rx_.start(domain)) { std::cerr << "[MapRecorderSystem] LIO receiver failed\n"; return false; }
    rx_started_ = true;

    uwb_started_ = uwb_.start(domain);   // non-fatal: no UWB -> no sidecar written on save
    if (!uwb_started_) std::cerr << "[MapRecorderSystem] UWB receiver failed — no sidecar will be written\n";

    // Fixed accumulation config (no env). To change the voxel leaf, edit MapRecorderConfig's default
    // in include/recorder/map_recorder.hpp.
    const MapRecorderConfig cfg;
    rec_.start(rx_, uwb_started_ ? &uwb_ : nullptr, cfg);

    std::cout << "[MapRecorderSystem] recording rt/cloud_registered_1 (voxel " << cfg.voxel_m
              << " m). Drive the environment, then Ctrl+C to save.\n";
    return true;
}

void MapRecorderSystem::stop() {
    rec_.stop();                                                   // join the accumulation thread first
    if (uwb_started_) { uwb_.stop(); uwb_started_ = false; }
    if (rx_started_)  { rx_.stop();  rx_started_  = false; }
}

void MapRecorderSystem::install_signal_handlers() {
    g_self = this;
    auto h = [](int) { if (g_self) g_self->request_quit(); };
    std::signal(SIGINT,  h);
    std::signal(SIGTERM, h);
}

} // namespace kist
