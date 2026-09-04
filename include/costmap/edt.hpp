#pragma once

// Exact 2-D squared Euclidean Distance Transform (Felzenszwalb & Huttenlocher
// 2012): two passes of the 1-D lower-envelope transform (rows, then columns).
// obstacle[i] != 0 marks an obstacle cell; returns, for every cell, the squared
// distance (in cell units) to the nearest obstacle. O(w*h). All-free input
// yields a large sentinel everywhere.

#include <cstdint>
#include <vector>

namespace kist {

std::vector<float> edt_squared(const std::vector<uint8_t>& obstacle, int w, int h);

} // namespace kist
