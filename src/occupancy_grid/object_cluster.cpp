#include "occupancy_grid/object_cluster.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace kist {

namespace {

// Connected-components on `mask`, emitting a DetectedObject per surviving blob.
// Class is the majority LABELLED class of the blob's cells (the camera labels
// only a surface, a minority of a cluster's cells): needs enough labels present
// AND a consensus among them, else kNoClass. Appends to `out`, ids from next_id.
void clusters_from_mask(const OccupancyGrid& g, const GridConfig& cfg,
                        const cv::Mat& mask, int& next_id,
                        std::vector<DetectedObject>& out) {
    cv::Mat labels, stats, centroids;
    const int nc = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
    if (nc <= 1) return;

    struct Acc { std::array<int, 256> hist{}; double psum = 0.0; };
    std::vector<Acc> acc(nc);
    for (int iy = 0; iy < g.n; ++iy)
        for (int ix = 0; ix < g.n; ++ix) {
            const int c = labels.at<int>(iy, ix);
            if (c <= 0) continue;
            const int idx = g.index(ix, iy);
            acc[c].hist[g.label[idx]]++;
            acc[c].psum += g.prob(idx);
        }

    const float cell = g.resolution;
    const float min_cells = cfg.cluster_min_area_m2 / (cell * cell);
    for (int c = 1; c < nc; ++c) {
        const int area = stats.at<int>(c, cv::CC_STAT_AREA);
        if (area < min_cells) continue;

        int best = kNoClass, best_n = 0, labeled = 0;
        for (int k = 0; k < kNoClass; ++k) {
            labeled += acc[c].hist[k];
            if (acc[c].hist[k] > best_n) { best_n = acc[c].hist[k]; best = k; }
        }
        // Drop near-fully-unknown blobs (LiDAR-only / noise) from the object
        // list — they stay in the occupancy grid for collision, just aren't
        // reported as detected objects.
        if (labeled < (1.f - cfg.cluster_drop_unknown_frac) * area) continue;

        DetectedObject o;
        o.id      = next_id++;
        o.n_cells = area;
        o.area_m2 = area * cell * cell;
        o.conf    = float(acc[c].psum / area);
        o.x = g.origin_x + float(centroids.at<double>(c, 0) + 0.5) * cell;
        o.y = g.origin_y + float(centroids.at<double>(c, 1) + 0.5) * cell;

        const int bx = stats.at<int>(c, cv::CC_STAT_LEFT);
        const int by = stats.at<int>(c, cv::CC_STAT_TOP);
        const int bw = stats.at<int>(c, cv::CC_STAT_WIDTH);
        const int bh = stats.at<int>(c, cv::CC_STAT_HEIGHT);
        o.min_x = g.origin_x + bx * cell;   o.max_x = g.origin_x + (bx + bw) * cell;
        o.min_y = g.origin_y + by * cell;   o.max_y = g.origin_y + (by + bh) * cell;

        const bool enough   = labeled >= cfg.cluster_min_labeled_frac * area;
        const bool dominant = best_n  >= cfg.cluster_label_min_frac   * labeled;
        o.class_id = (best_n > 0 && enough && dominant) ? uint8_t(best) : kNoClass;

        out.push_back(o);
    }
}

}  // namespace

ObjectList extract_clusters(const OccupancyGrid& g, const GridConfig& cfg,
                            const cv::Mat& instance_map) {
    ObjectList out;
    out.stamp_ns = g.stamp_ns;
    if (g.empty()) return out;

    // Occupied mask (row = iy, col = ix — same indexing as the grid).
    cv::Mat occ(g.n, g.n, CV_8U, cv::Scalar(0));
    for (int iy = 0; iy < g.n; ++iy)
        for (int ix = 0; ix < g.n; ++ix)
            if (g.prob(g.index(ix, iy)) > cfg.occ_threshold)
                occ.at<uint8_t>(iy, ix) = 255;

    if (cfg.cluster_morph_cells > 0) {
        const int k = 2 * cfg.cluster_morph_cells + 1;
        cv::morphologyEx(occ, occ, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k}));
    }

    const float cell = g.resolution;
    const float min_cells = cfg.cluster_min_area_m2 / (cell * cell);
    int     next_id = 0;
    cv::Mat claimed(g.n, g.n, CV_8U, cv::Scalar(0));

    // 1) Instance split: every current-frame instance (ANY class) seeds a
    // min-enclosing-circle footprint that absorbs the cells inside — so touching
    // objects of the same class (two people, two chairs) or different classes
    // separate, since instance seg gives each its own id. Smaller instances are
    // claimed first so a big object's circle can't swallow a small neighbour.
    if (!instance_map.empty()) {
        std::map<int, std::vector<cv::Point>> seeds;
        for (int iy = 0; iy < g.n; ++iy)
            for (int ix = 0; ix < g.n; ++ix) {
                if (!occ.at<uint8_t>(iy, ix)) continue;
                const int inst = instance_map.at<int>(iy, ix);
                if (inst >= 0) seeds[inst].push_back(cv::Point(ix, iy));
            }
        std::vector<const std::vector<cv::Point>*> order;
        for (auto& kv : seeds) order.push_back(&kv.second);
        std::sort(order.begin(), order.end(),
                  [](auto a, auto b) { return a->size() < b->size(); });   // small first
        const float pad = cfg.cluster_footprint_pad_m / cell;   // grow to absorb behind the surface
        for (const auto* cells : order) {
            if (cells->size() < 3) continue;
            // Oriented footprint: min-area rotated box -> the object's direction +
            // aspect, so an elongated object gets a long thin ELLIPSE (not a big
            // circle that swallows perpendicular neighbours).
            cv::RotatedRect rr = cv::minAreaRect(*cells);
            rr.size.width  += 2 * pad;
            rr.size.height += 2 * pad;
            cv::Mat region(g.n, g.n, CV_8U, cv::Scalar(0));
            cv::ellipse(region, rr, 255, cv::FILLED);    // ellipse inscribed in the oriented box
            cv::bitwise_and(region, occ, region);        // absorb the cells inside
            cv::subtract(region, claimed, region);       // don't double-claim
            if (cv::countNonZero(region) < min_cells) continue;
            clusters_from_mask(g, cfg, region, next_id, out.objects);
            cv::bitwise_or(claimed, region, claimed);
        }
    }

    // 2) Remainder: everything unclaimed (LiDAR-only, undetected) -> normal CC.
    cv::Mat rest;
    cv::subtract(occ, claimed, rest);
    clusters_from_mask(g, cfg, rest, next_id, out.objects);

    return out;
}

} // namespace kist
