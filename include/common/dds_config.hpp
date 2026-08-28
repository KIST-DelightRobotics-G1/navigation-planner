#pragma once

// Routes the DDS transport config (config/cyclonedds.xml: network interface
// + socket-buffer tuning) into CycloneDDS for a runner process.
//
// Kept identical to kist-ext-sensor-io's common/dds_config.hpp (same pairing as
// config.hpp / data_buffer.hpp) so every runner in either repo uses one helper.
//
// Why this exists: ChannelFactory::Init(domain, iface) with a non-empty
// iface substitutes iface into the SDK's embedded DDS config template and
// creates the domain from that — CYCLONEDDS_URI is then SILENTLY ignored,
// and with it every knob in config/cyclonedds.xml (measured: sockets stay
// at Cyclone's 1 MiB default). So a runner must do two things:
//   1. route the XML via CYCLONEDDS_URI — done here, with loud validation
//      because the failure mode is silent;
//   2. pass an EMPTY network interface into every start()/Init() — the
//      XML's <NetworkInterface name=.../> names the NIC instead.
//
// Precedence: a pre-set CYCLONEDDS_URI (e.g. the Docker image ENV) wins;
// otherwise `unitree.dds_config` (default config/cyclonedds.xml) is
// resolved, validated, and exported. Returns false (after printing why)
// when neither is usable. Call before the first subscriber/publisher start.

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace kist {

inline bool apply_dds_config(const YAML::Node& root) {
    if (const char* prev = std::getenv("CYCLONEDDS_URI"); prev && *prev) {
        std::printf("[dds_config] CYCLONEDDS_URI=%s (pre-set)\n", prev);
        return true;
    }
    const auto unitree = root["unitree"];
    const std::string path = unitree
        ? unitree["dds_config"].as<std::string>("config/cyclonedds.xml")
        : "config/cyclonedds.xml";
    std::error_code ec;
    const auto abs = std::filesystem::absolute(path, ec);
    if (ec || !std::filesystem::exists(abs)) {
        std::fprintf(stderr,
            "[dds_config] DDS config not found: %s — the network interface and "
            "socket tuning live there (unitree.dds_config), not in config.yaml\n",
            path.c_str());
        return false;
    }
    const std::string uri = "file://" + abs.string();
    setenv("CYCLONEDDS_URI", uri.c_str(), 1);
    std::printf("[dds_config] CYCLONEDDS_URI=%s\n", uri.c_str());
    return true;
}

} // namespace kist
