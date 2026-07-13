#pragma once

#include "common/pointcloud_frame.hpp"

#include <unitree/idl/ros2/PointCloud2_.hpp>

namespace kist {

// PointCloud2 (DDS IDL, as relayed by the robot) -> xyz frame.
// Pure function: locates the FLOAT32 x/y/z fields by name, walks the
// binary blob by point_step, drops non-finite points. Returns false when
// the x/y/z fields are missing (frame_out untouched).
bool decode_pointcloud2(const sensor_msgs::msg::dds_::PointCloud2_& msg,
                        PointCloudXYZFrame& frame_out);

} // namespace kist
