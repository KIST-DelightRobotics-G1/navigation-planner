#pragma once

// Clearance field + medial axis from a costmap — the geometry the planner's centreline
// smoothing is built on, exposed for visualization/debugging.
//
//   clearance_field: distance (m) from every cell to the nearest impassable cell. Small
//                    near walls, large in the middle of a corridor. Its "valleys" are the
//                    walls; its RIDGE is the corridor centreline.
//   medial_axis:     the ridge cells (local maxima of the clearance field) = the centreline
//                    skeleton the smoother pulls toward. Corners are simply where it bends.

#include "route_planner/costmap_builder/costmap.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace kist {

// Per-cell distance (m) to the nearest cell with cost >= lethal. Row-major (iy*n + ix).
std::vector<float> clearance_field(const Costmap& cm, uint8_t lethal = 254);

// Ridge of the clearance field: cells that are a local maximum along at least one axis and
// clear by at least min_clearance_m. Returned as odom (x,y) point centres — the centreline.
std::vector<std::pair<float, float>>
medial_axis(const Costmap& cm, const std::vector<float>& clr, float min_clearance_m = 0.25f);

} // namespace kist
