#include "processing/pointcloud_processor.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <utility>

namespace kist {

namespace {

inline void transform_point(float& x, float& y, float& z,
                            const PointCloudTransformOptions& t) noexcept {
    x = t.offset_x + (t.invert_x ? -x : x);
    y = t.offset_y + (t.invert_y ? -y : y);
    z = t.offset_z + (t.invert_z ? -z : z);
}

inline bool accept_point(float x, float y, float z,
                         const PointCloudFilterOptions& f) noexcept {
    if (f.enable_x && (x < f.min_x || x > f.max_x)) return false;
    if (f.enable_y && (y < f.min_y || y > f.max_y)) return false;
    if (f.enable_z && (z < f.min_z || z > f.max_z)) return false;

    if (f.enable_radius && (x * x + y * y) < f.min_radius * f.min_radius)
        return false;

    return true;
}

template <typename T>
T get_or(const YAML::Node& node, const char* key, T fallback) {
    return node[key] ? node[key].as<T>() : fallback;
}

} // namespace

PointCloudProcessorOptions pointcloud_processor_options_from_yaml(const YAML::Node& section) {
    PointCloudProcessorOptions opt;
    if (!section)
        return opt;

    if (const auto t = section["transform"]) {
        opt.transform.enabled  = get_or(t, "enabled",  opt.transform.enabled);
        opt.transform.offset_x = get_or(t, "offset_x", opt.transform.offset_x);
        opt.transform.offset_y = get_or(t, "offset_y", opt.transform.offset_y);
        opt.transform.offset_z = get_or(t, "offset_z", opt.transform.offset_z);
        opt.transform.invert_x = get_or(t, "invert_x", opt.transform.invert_x);
        opt.transform.invert_y = get_or(t, "invert_y", opt.transform.invert_y);
        opt.transform.invert_z = get_or(t, "invert_z", opt.transform.invert_z);
    }

    if (const auto f = section["filter"]) {
        opt.filter.enabled       = get_or(f, "enabled",       opt.filter.enabled);
        opt.filter.enable_x      = get_or(f, "enable_x",      opt.filter.enable_x);
        opt.filter.min_x         = get_or(f, "min_x",         opt.filter.min_x);
        opt.filter.max_x         = get_or(f, "max_x",         opt.filter.max_x);
        opt.filter.enable_y      = get_or(f, "enable_y",      opt.filter.enable_y);
        opt.filter.min_y         = get_or(f, "min_y",         opt.filter.min_y);
        opt.filter.max_y         = get_or(f, "max_y",         opt.filter.max_y);
        opt.filter.enable_z      = get_or(f, "enable_z",      opt.filter.enable_z);
        opt.filter.min_z         = get_or(f, "min_z",         opt.filter.min_z);
        opt.filter.max_z         = get_or(f, "max_z",         opt.filter.max_z);
        opt.filter.enable_radius = get_or(f, "enable_radius", opt.filter.enable_radius);
        opt.filter.min_radius    = get_or(f, "min_radius",    opt.filter.min_radius);
    }

    return opt;
}

void validate_processor_options(const PointCloudProcessorOptions& options) {
    const auto& f = options.filter;

    if (f.enable_x && f.min_x > f.max_x)
        throw std::invalid_argument("filter.min_x must not exceed filter.max_x");

    if (f.enable_y && f.min_y > f.max_y)
        throw std::invalid_argument("filter.min_y must not exceed filter.max_y");

    if (f.enable_z && f.min_z > f.max_z)
        throw std::invalid_argument("filter.min_z must not exceed filter.max_z");
}

PointCloudProcessor::PointCloudProcessor(PointCloudProcessorOptions options)
    : options_(std::move(options)) {
    validate_processor_options(options_);
}

void PointCloudProcessor::process(UnitreePointCloud& frame) const {
    const auto& t = options_.transform;
    const auto& f = options_.filter;
    if (!t.enabled && !f.enabled)
        return;

    std::size_t w = 0;
    for (std::size_t r = 0; r + 2 < frame.xyz.size(); r += 3) {
        float x = frame.xyz[r];
        float y = frame.xyz[r + 1];
        float z = frame.xyz[r + 2];

        if (t.enabled)
            transform_point(x, y, z, t);

        if (f.enabled && !accept_point(x, y, z, f))
            continue;

        frame.xyz[w]     = x;
        frame.xyz[w + 1] = y;
        frame.xyz[w + 2] = z;
        w += 3;
    }
    frame.xyz.resize(w);
}

} // namespace kist
