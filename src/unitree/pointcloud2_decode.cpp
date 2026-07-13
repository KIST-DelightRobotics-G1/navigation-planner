#include "unitree/pointcloud2_decode.hpp"

#include <cmath>
#include <cstring>
#include <optional>

namespace kist {

namespace {

constexpr uint8_t kFloat32 = 7;  // sensor_msgs/PointField FLOAT32

struct XYZOffsets {
    uint32_t x{0}, y{0}, z{0};
};

std::optional<XYZOffsets> find_xyz_offsets(
    const std::vector<sensor_msgs::msg::dds_::PointField_>& fields) {
    XYZOffsets off;
    bool fx = false, fy = false, fz = false;
    for (const auto& f : fields) {
        if (f.datatype() != kFloat32)
            continue;
        if (f.name() == "x") { off.x = f.offset(); fx = true; }
        else if (f.name() == "y") { off.y = f.offset(); fy = true; }
        else if (f.name() == "z") { off.z = f.offset(); fz = true; }
    }
    if (!fx || !fy || !fz)
        return std::nullopt;
    return off;
}

} // namespace

bool decode_pointcloud2(const sensor_msgs::msg::dds_::PointCloud2_& msg,
                        PointCloudXYZFrame& frame_out) {
    const auto off = find_xyz_offsets(msg.fields());
    if (!off)
        return false;

    frame_out.stamp_ns = int64_t(msg.header().stamp().sec()) * 1000000000LL +
                         msg.header().stamp().nanosec();
    frame_out.frame_id = msg.header().frame_id();
    frame_out.xyz.clear();

    const std::size_t n = std::size_t(msg.width()) * msg.height();
    const auto& data = msg.data();
    frame_out.xyz.reserve(n * 3);

    for (std::size_t i = 0; i < n; ++i) {
        const uint8_t* p = data.data() + i * msg.point_step();
        float x, y, z;
        std::memcpy(&x, p + off->x, sizeof(float));
        std::memcpy(&y, p + off->y, sizeof(float));
        std::memcpy(&z, p + off->z, sizeof(float));
        if (!std::isfinite(x + y + z))
            continue;
        frame_out.xyz.insert(frame_out.xyz.end(), {x, y, z});
    }
    return true;
}

} // namespace kist
