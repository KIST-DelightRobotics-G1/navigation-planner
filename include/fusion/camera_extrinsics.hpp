#pragma once

// Camera -> robot-base extrinsics for the fused cloud. fuse_depth_semantic()
// emits points in the RealSense OPTICAL frame (+X right, +Y down, +Z forward);
// this rigid transform lifts them into the ROBOT BASE frame (+X forward,
// +Y left, +Z up), so the floor is level, heights are real, and the cloud
// shares the LiDAR's frame:
//     p_robot = R * p_cam + t
// Built from the mount pose: position (x, y, height) + orientation
// (pitch/yaw/roll). pitch is the downward tilt from horizontal (positive =
// camera looks down). These are an estimate — tune live in test_fusion_viewer
// and copy the values into config.

#include "fusion/depth_fusion.hpp"

#include <cmath>
#include <cstdint>

namespace kist {

struct CameraExtrinsics {
    float R[9] = {1,0,0, 0,1,0, 0,0,1};  // row-major 3x3, cam-optical -> robot
    float t[3] = {0,0,0};                 // camera position in robot frame (m)
};

namespace detail {
// out = a * b  (row-major 3x3)
inline void mat3_mul(const float a[9], const float b[9], float out[9]) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out[r*3+c] = a[r*3+0]*b[0*3+c] + a[r*3+1]*b[1*3+c] + a[r*3+2]*b[2*3+c];
}
}  // namespace detail

// Extrinsics from the mount pose. Angles in degrees; pitch = downward tilt.
inline CameraExtrinsics make_camera_extrinsics(
        float x, float y, float height,
        float pitch_deg, float yaw_deg = 0.f, float roll_deg = 0.f) {
    constexpr float kDeg = float(M_PI) / 180.f;
    const float p = pitch_deg * kDeg, yw = yaw_deg * kDeg, rl = roll_deg * kDeg;

    // Optical -> robot when the camera looks forward, level:
    //   Z_cam(forward)->X_robot, X_cam(right)->-Y_robot, Y_cam(down)->-Z_robot
    const float R0[9] = { 0, 0, 1,
                         -1, 0, 0,
                          0,-1, 0 };
    // Mount orientation in the robot frame: yaw(Z) * pitch(Y) * roll(X).
    // Ry(+pitch) tilts the forward axis downward (verified: X_robot -> -Z).
    const float cy = std::cos(yw), sy = std::sin(yw);
    const float cp = std::cos(p),  sp = std::sin(p);
    const float cr = std::cos(rl), sr = std::sin(rl);
    const float Rz[9] = { cy,-sy, 0,  sy, cy, 0,  0, 0, 1 };
    const float Ry[9] = { cp, 0, sp,   0, 1, 0, -sp, 0, cp };
    const float Rx[9] = { 1, 0, 0,   0, cr,-sr,  0, sr, cr };

    float Rzy[9], Rmount[9];
    detail::mat3_mul(Rz, Ry, Rzy);
    detail::mat3_mul(Rzy, Rx, Rmount);

    CameraExtrinsics e;
    detail::mat3_mul(Rmount, R0, e.R);
    e.t[0] = x; e.t[1] = y; e.t[2] = height;
    return e;
}

// Transform a camera-frame cloud into the robot frame (labels carried through).
// out may not alias in. Reuses out's storage.
inline void transform_cloud(const LabeledCloud& in, const CameraExtrinsics& e,
                            LabeledCloud& out) {
    out.stamp_ns = in.stamp_ns;
    const size_t n = in.size();
    out.xyz.resize(n * 3);
    out.label.resize(n);
    const float* R = e.R; const float* t = e.t;
    for (size_t i = 0; i < n; ++i) {
        const float X = in.xyz[3*i], Y = in.xyz[3*i+1], Z = in.xyz[3*i+2];
        out.xyz[3*i]   = R[0]*X + R[1]*Y + R[2]*Z + t[0];
        out.xyz[3*i+1] = R[3]*X + R[4]*Y + R[5]*Z + t[1];
        out.xyz[3*i+2] = R[6]*X + R[7]*Y + R[8]*Z + t[2];
        out.label[i]   = in.label[i];
    }
}

} // namespace kist
