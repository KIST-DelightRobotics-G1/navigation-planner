#include "planner/costmap/edt.hpp"

#include <algorithm>
#include <limits>

namespace kist {

namespace {
constexpr float kInf = 1e18f;   // "no obstacle in this line" — finite, no overflow

// 1-D squared distance transform: d[q] = min_j { (q-j)^2 + f[j] }.
// v (size n+1) and z (size n+2) are caller scratch. Lower-envelope of parabolas.
void dt1d(const float* f, float* d, int* v, float* z, int n) {
    int k = 0;
    v[0] = 0;
    z[0] = -std::numeric_limits<float>::infinity();
    z[1] =  std::numeric_limits<float>::infinity();
    for (int q = 1; q < n; ++q) {
        float s;
        while (true) {
            const int vk = v[k];
            s = ((f[q] + float(q) * q) - (f[vk] + float(vk) * vk)) / (2.0f * (q - vk));
            if (s > z[k]) break;
            --k;
        }
        ++k;
        v[k]     = q;
        z[k]     = s;
        z[k + 1] = std::numeric_limits<float>::infinity();
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < float(q)) ++k;
        const float diff = float(q - v[k]);
        d[q] = diff * diff + f[v[k]];
    }
}
}  // namespace

std::vector<float> edt_squared(const std::vector<uint8_t>& obstacle, int w, int h) {
    const int N = w * h;
    const int maxd = std::max(w, h);
    std::vector<float> row(maxd), out(maxd);
    std::vector<int>   v(maxd + 1);
    std::vector<float> z(maxd + 2);

    std::vector<float> d(N);

    // Pass 1: rows -> squared horizontal distance.
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c)
            row[c] = obstacle[r * w + c] ? 0.0f : kInf;
        dt1d(row.data(), out.data(), v.data(), z.data(), w);
        for (int c = 0; c < w; ++c) d[r * w + c] = out[c];
    }
    // Pass 2: columns over the row result -> exact 2-D squared distance.
    for (int c = 0; c < w; ++c) {
        for (int r = 0; r < h; ++r) row[r] = d[r * w + c];
        dt1d(row.data(), out.data(), v.data(), z.data(), h);
        for (int r = 0; r < h; ++r) d[r * w + c] = out[r];
    }
    return d;
}

} // namespace kist
