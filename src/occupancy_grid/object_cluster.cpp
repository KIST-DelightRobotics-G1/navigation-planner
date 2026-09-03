#include "occupancy_grid/object_cluster.hpp"

#include <opencv2/imgproc.hpp>

#include <array>

namespace kist {

ObjectList extract_clusters(const OccupancyGrid& g, const GridConfig& cfg) {
    ObjectList out;
    out.stamp_ns = g.stamp_ns;
    if (g.empty()) return out;

    // Binary occupied mask. Row = iy (Y axis), col = ix (X axis) — same indexing
    // as the grid, so connectedComponents coords map straight back to cells.
    cv::Mat mask(g.n, g.n, CV_8U, cv::Scalar(0));
    for (int iy = 0; iy < g.n; ++iy)
        for (int ix = 0; ix < g.n; ++ix)
            if (g.prob(g.index(ix, iy)) > cfg.occ_threshold)
                mask.at<uint8_t>(iy, ix) = 255;

    if (cfg.cluster_morph_cells > 0) {
        const int k = 2 * cfg.cluster_morph_cells + 1;
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k}));
    }

    cv::Mat labels, stats, centroids;
    const int nc = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
    if (nc <= 1) return out;   // only background

    // One pass over the labelled cells: majority class + mean P(occ) per component.
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
    int next_id = 0;
    for (int c = 1; c < nc; ++c) {
        const int area = stats.at<int>(c, cv::CC_STAT_AREA);
        if (area < min_cells) continue;

        DetectedObject o;
        o.id      = next_id++;
        o.n_cells = area;
        o.area_m2 = area * cell * cell;
        o.conf    = float(acc[c].psum / area);

        // centroid (col=ix, row=iy) -> world
        o.x = g.origin_x + float(centroids.at<double>(c, 0) + 0.5) * cell;
        o.y = g.origin_y + float(centroids.at<double>(c, 1) + 0.5) * cell;

        const int bx = stats.at<int>(c, cv::CC_STAT_LEFT);
        const int by = stats.at<int>(c, cv::CC_STAT_TOP);
        const int bw = stats.at<int>(c, cv::CC_STAT_WIDTH);
        const int bh = stats.at<int>(c, cv::CC_STAT_HEIGHT);
        o.min_x = g.origin_x + bx * cell;
        o.max_x = g.origin_x + (bx + bw) * cell;
        o.min_y = g.origin_y + by * cell;
        o.max_y = g.origin_y + (by + bh) * cell;

        // Class among the LABELLED cells (camera labels only a surface, so a
        // minority of a cluster's cells): need enough labels present AND the
        // top class to dominate the labelled cells; else the blob stays unknown.
        int best = kNoClass, best_n = 0, labeled = 0;
        for (int k = 0; k < kNoClass; ++k) {
            labeled += acc[c].hist[k];
            if (acc[c].hist[k] > best_n) { best_n = acc[c].hist[k]; best = k; }
        }
        const bool enough   = labeled >= cfg.cluster_min_labeled_frac * area;
        const bool dominant = best_n  >= cfg.cluster_label_min_frac   * labeled;
        o.class_id = (best_n > 0 && enough && dominant) ? uint8_t(best) : kNoClass;

        out.objects.push_back(o);
    }
    return out;
}

} // namespace kist
