#pragma once

// Single-file config access (config/config.yaml, one section per stage) —
// same pattern as kist-gearsonic-inference: load once at the assembly
// point, then each stage reads its own section from root().

#include <yaml-cpp/yaml.h>
#include <string>

namespace kist {

class Config {
public:
    static Config& instance() {
        static Config inst;
        return inst;
    }

    void load(const std::string& path) {
        root_ = YAML::LoadFile(path);
    }

    const YAML::Node& root() const { return root_; }

private:
    Config() = default;
    YAML::Node root_;
};

} // namespace kist
