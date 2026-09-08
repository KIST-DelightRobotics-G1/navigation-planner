#include "transforms/rigid.hpp"

#include <cmath>

namespace kist {

Rigid rigid_mul(const Rigid& a, const Rigid& b) {
    Rigid o;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            float s = 0.f;
            for (int k = 0; k < 3; ++k) s += a.r[r*3+k] * b.r[k*3+c];
            o.r[r*3+c] = s;
        }
    for (int r = 0; r < 3; ++r)
        o.t[r] = a.r[r*3+0]*b.t[0] + a.r[r*3+1]*b.t[1] + a.r[r*3+2]*b.t[2] + a.t[r];
    return o;
}

Rigid rigid_inv(const Rigid& a) {
    Rigid o;
    for (int r = 0; r < 3; ++r)          // R^T
        for (int c = 0; c < 3; ++c)
            o.r[r*3+c] = a.r[c*3+r];
    for (int r = 0; r < 3; ++r)          // -R^T t
        o.t[r] = -(o.r[r*3+0]*a.t[0] + o.r[r*3+1]*a.t[1] + o.r[r*3+2]*a.t[2]);
    return o;
}

Rigid rot_x(float a, float tx, float ty, float tz) {
    const float c = std::cos(a), s = std::sin(a);
    return { {1,0,0, 0,c,-s, 0,s,c}, {tx,ty,tz} };
}
Rigid rot_y(float a, float tx, float ty, float tz) {
    const float c = std::cos(a), s = std::sin(a);
    return { {c,0,s, 0,1,0, -s,0,c}, {tx,ty,tz} };
}
Rigid rot_z(float a, float tx, float ty, float tz) {
    const float c = std::cos(a), s = std::sin(a);
    return { {c,-s,0, s,c,0, 0,0,1}, {tx,ty,tz} };
}

Rigid from_rpy(float roll, float pitch, float yaw, float tx, float ty, float tz) {
    // R = Rz(yaw) * Ry(pitch) * Rx(roll), translation applied as the origin.
    Rigid rot = rigid_mul(rigid_mul(rot_z(yaw), rot_y(pitch)), rot_x(roll));
    rot.t[0] = tx; rot.t[1] = ty; rot.t[2] = tz;
    return rot;
}

} // namespace kist
