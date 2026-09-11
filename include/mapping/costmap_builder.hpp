#pragma once

// Turn an ObstacleGrid into a Costmap: occupied cells (P > occ_threshold) -> 8-neighbour
// speckle filter -> EDT -> distance->cost LUT. A stateless-ish module (caches the LUT); a
// consumer (the perception worker) calls build() per grid. No OpenCV, no semantic/dynamic
// layer — LiDAR-only for now.

#include "mapping/costmap.hpp"
#include "mapping/obstacle_grid.hpp"   // ObstacleGrid, ObstacleGridConfig (occ_threshold)

namespace kist {

class CostmapBuilder {
public:
    // gcfg supplies occ_threshold (what counts as occupied); cfg the radii / cost shape.
    Costmap build(const ObstacleGrid& g, const ObstacleGridConfig& gcfg, const CostmapConfig& cfg);

private:
    void build_lut(float resolution, const CostmapConfig& cfg);

    std::vector<uint8_t> lut_;    // cost indexed by squared distance (cell units)
    float                lut_res_ = -1.0f;
};

} // namespace kist
