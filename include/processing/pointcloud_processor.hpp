#pragma once

#include "unitree/unitree_pointcloud.hpp"

namespace YAML {
class Node;
}

namespace kist {

// Per-point cloud processing stage: transform (axis invert + offset),
// then filter (axis range + XY radius) on the transformed coordinates.
// Runs in place in a single pass — cheap enough to execute inside the
// reader's DDS receive callback via set_processor; it owns no thread.
//
// Everything defaults to disabled: a default-constructed processor is a
// passthrough.

struct PointCloudTransformOptions {
    bool enabled = false;

    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;

    bool invert_x = false;
    bool invert_y = false;
    bool invert_z = false;
};

struct PointCloudFilterOptions {
    bool enabled = false;

    bool  enable_x = false;
    float min_x = 0.0f;
    float max_x = 0.0f;

    bool  enable_y = false;
    float min_y = 0.0f;
    float max_y = 0.0f;

    bool  enable_z = false;
    float min_z = 0.0f;
    float max_z = 0.0f;

    bool  enable_radius = false;
    float min_radius = 0.0f;  // points within this XY radius of the origin are removed
};

struct PointCloudProcessorOptions {
    PointCloudTransformOptions transform;
    PointCloudFilterOptions    filter;
};

// Reads the `pointcloud_processor` section of config.yaml (pass
// Config::instance().root()["pointcloud_processor"]). Missing keys keep
// their defaults, so a partial section is fine.
PointCloudProcessorOptions pointcloud_processor_options_from_yaml(const YAML::Node& section);

// Throws std::invalid_argument when an enabled axis range has min > max.
void validate_processor_options(const PointCloudProcessorOptions& options);

class PointCloudProcessor {
public:
    // Validates the options (see validate_processor_options).
    explicit PointCloudProcessor(PointCloudProcessorOptions options);

    // Transform first, then filter on the transformed coordinates.
    // Surviving points are compacted in place; stamp/frame_id untouched.
    void process(UnitreePointCloud& frame) const;

private:
    PointCloudProcessorOptions options_;
};

} // namespace kist
