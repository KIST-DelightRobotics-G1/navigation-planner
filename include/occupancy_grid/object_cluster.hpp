#pragma once

// Object clustering on the occupancy grid — group connected occupied cells into
// discrete objects (BEV connected-components). The GROUPING is geometric (works
// on occupancy, i.e. LiDAR + camera together); segmentation only TAGS each
// resulting cluster with the majority class of its cells. A stateless Module:
// the OccupancyGridBuilder calls extract_clusters() inline on its own thread
// (clustering is light + strictly downstream of the grid, so no extra thread).

#include "occupancy_grid/occupancy_grid.hpp"   // OccupancyGrid, GridConfig, kNoClass

#include <cstdint>
#include <vector>

namespace kist {

// One clustered object, in world coordinates.
struct DetectedObject {
    int     id = -1;                    // cluster id this frame (tracking later)
    float   x = 0.f, y = 0.f;           // centroid (m, world)
    float   min_x = 0.f, min_y = 0.f;   // bbox (m, world)
    float   max_x = 0.f, max_y = 0.f;
    float   area_m2 = 0.f;
    uint8_t class_id = kNoClass;        // majority class over the cluster's cells
    float   conf = 0.f;                 // mean P(occ) over the cluster
    int     n_cells = 0;
};

struct ObjectList {
    int64_t stamp_ns = 0;
    std::vector<DetectedObject> objects;
    bool empty() const { return objects.empty(); }
    size_t size() const { return objects.size(); }
};

// Cluster the grid's occupied cells (P(occ) > occ_threshold) into objects.
// Optional morphological close bridges small gaps; blobs below cluster_min_area
// are dropped. Each object's class is the majority labeled class of its cells
// (kNoClass if none labeled — e.g. a LiDAR-only obstacle).
ObjectList extract_clusters(const OccupancyGrid& g, const GridConfig& cfg);

} // namespace kist
