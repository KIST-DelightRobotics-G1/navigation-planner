#include "frames/sensor_extrinsics.hpp"

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

#include <cstddef>
#include <exception>

namespace kist {

// URDF rpy = fixed-axis XYZ = intrinsic ZYX: R = Rz(yaw) Ry(pitch) Rx(roll).
static Eigen::Quaterniond rpy_to_quat(double roll, double pitch, double yaw) {
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
}

std::optional<SensorExtrinsics> load_sensor_extrinsics(const std::string& path,
                                                       std::string* err) {
    auto fail = [&](const std::string& m) -> std::optional<SensorExtrinsics> {
        if (err) *err = m;
        return std::nullopt;
    };

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        return fail("cannot load " + path + ": " + e.what());
    }

    const YAML::Node list = root["transforms"];
    if (!list || !list.IsSequence())
        return fail("missing or non-sequence 'transforms' in " + path);

    SensorExtrinsics out;
    for (std::size_t i = 0; i < list.size(); ++i) {
        const YAML::Node n   = list[i];
        const std::string ctx = "transforms[" + std::to_string(i) + "]";

        if (!n["parent"] || !n["child"])
            return fail(ctx + ": missing parent/child");

        const auto ps = n["parent"].as<std::string>();
        const auto cs = n["child"].as<std::string>();
        const auto parent = frame_from_name(ps);
        const auto child  = frame_from_name(cs);
        if (!parent) return fail(ctx + ": unknown parent frame '" + ps + "'");
        if (!child)  return fail(ctx + ": unknown child frame '"  + cs + "'");
        if (*parent == *child) return fail(ctx + ": parent == child (" + ps + ")");

        for (const auto& e : out.transforms)
            if (e.parent == *parent && e.child == *child)
                return fail(ctx + ": duplicate edge " + ps + " -> " + cs);

        StaticTransform st;
        st.parent = *parent;
        st.child  = *child;

        const YAML::Node t = n["translation"];
        st.T_parent_child.translation = Eigen::Vector3d(
            t ? t["x"].as<double>(0.0) : 0.0,
            t ? t["y"].as<double>(0.0) : 0.0,
            t ? t["z"].as<double>(0.0) : 0.0);

        const YAML::Node r = n["rotation_rpy"];
        st.T_parent_child.rotation = rpy_to_quat(
            r ? r["roll"].as<double>(0.0)  : 0.0,
            r ? r["pitch"].as<double>(0.0) : 0.0,
            r ? r["yaw"].as<double>(0.0)   : 0.0);

        out.transforms.push_back(st);
    }

    return out;
}

} // namespace kist
