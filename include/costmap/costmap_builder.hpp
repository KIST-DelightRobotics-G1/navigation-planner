#pragma once

// Turn an OccupancyGrid into a Costmap: occupied cells (+ an extra dilation
// around dynamic/person cells) -> EDT -> distance->cost LUT. A stateless-ish
// Module (caches the LUT); a consumer (the planner) calls build() per grid.

#include "costmap/costmap.hpp"
#include "occupancy_grid/occupancy_grid.hpp"   // OccupancyGrid, GridConfig (occ_threshold, is_dynamic)

#include <opencv2/core.hpp>   // cv::Mat (dynamic footprint mask)

namespace kist {

class CostmapBuilder {
public:
    // gcfg supplies occ_threshold + is_dynamic; cfg the radii / cost shape.
    // dynamic_mask (n x n CV_8U from clustering) marks dynamic-object footprints;
    // when cfg.ignore_dynamic, those cells are left out of the costmap (people
    // are handled by stop-and-ask, not avoidance). Empty mask -> fall back to the
    // grid's per-cell dynamic label.
    Costmap build(const OccupancyGrid& g, const GridConfig& gcfg, const CostmapConfig& cfg,
                  const cv::Mat& dynamic_mask = cv::Mat());

private:
    void build_lut(float resolution, const CostmapConfig& cfg);

    std::vector<uint8_t> lut_;    // cost indexed by squared distance (cell units)
    float                lut_res_ = -1.0f;
};

} // namespace kist
