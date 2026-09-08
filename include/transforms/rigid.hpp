#pragma once

// Rigid-body transform (SE(3)) — the shared transform type for the frame tree
// (map / odom / pelvis / torso / lidar / camera). p' = R p + t, R row-major 3x3.
// Axis convention: REP-103 (x forward, y left, z up; roll=x, pitch=y, yaw=z).

namespace kist {

struct Rigid {
    float r[9] = {1,0,0, 0,1,0, 0,0,1};   // row-major rotation
    float t[3] = {0,0,0};                  // translation

    // Transform a point: out = R * p + t.
    void apply(float x, float y, float z, float& ox, float& oy, float& oz) const {
        ox = r[0]*x + r[1]*y + r[2]*z + t[0];
        oy = r[3]*x + r[4]*y + r[5]*z + t[1];
        oz = r[6]*x + r[7]*y + r[8]*z + t[2];
    }
};

Rigid rigid_mul(const Rigid& a, const Rigid& b);   // a * b
Rigid rigid_inv(const Rigid& a);                   // inverse

// Elementary transforms (rotation about an axis at a fixed translation origin).
Rigid rot_x(float rad, float tx = 0, float ty = 0, float tz = 0);
Rigid rot_y(float rad, float tx = 0, float ty = 0, float tz = 0);
Rigid rot_z(float rad, float tx = 0, float ty = 0, float tz = 0);

// Fixed transform from roll/pitch/yaw (URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll))
// plus a translation.
Rigid from_rpy(float roll, float pitch, float yaw, float tx = 0, float ty = 0, float tz = 0);

} // namespace kist
