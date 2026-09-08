#pragma once

// Camera -> robot-base extrinsics for the labeled cloud. build_labeled_cloud()
// emits points in the RealSense OPTICAL frame (+X right, +Y down, +Z forward);
// this rigid transform lifts them into the ROBOT BASE frame (+X forward,
// +Y left, +Z up), so the floor is level, heights are real, and the cloud
// shares the LiDAR's frame:
//     p_robot = R * p_cam + t
// Built from the mount pose: position (x, y, height) + orientation
// (pitch/yaw/roll). pitch is the downward tilt from horizontal (positive =
// camera looks down). These are an estimate — tune live in test_fusion_viewer
// and copy the values into config.

#include "labeled_cloud/labeled_cloud.hpp"   // LabeledCloud

namespace kist {

struct CameraExtrinsics {
    float R[9] = {1,0,0, 0,1,0, 0,0,1};  // row-major 3x3, cam-optical -> robot
    float t[3] = {0,0,0};                 // camera position in robot frame (m)
};

// Extrinsics from the mount pose. Angles in degrees; pitch = downward tilt.
CameraExtrinsics make_camera_extrinsics(float x, float y, float height,
                                        float pitch_deg, float yaw_deg = 0.f,
                                        float roll_deg = 0.f);

// Transform a camera-frame cloud into the robot frame (labels carried through).
// out may not alias in. Reuses out's storage.
void transform_cloud(const LabeledCloud& in, const CameraExtrinsics& e,
                     LabeledCloud& out);

} // namespace kist
