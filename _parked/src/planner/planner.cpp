#include "planner/planner.hpp"

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace kist {

namespace {

// A* reruns from the degradation backstop are capped to this cadence, so a path
// stuck in a high-cost zone (no better route exists) can't rerun the search every
// frame. Internal guard, not a tuning knob — the check itself runs every cycle.
constexpr float kBackstopMinInterval_s = 0.1f;   // 10 Hz

// Any waypoint cell now impassable — or off the (recentred) map? Then the
// committed path is no longer valid and must be replanned. O(path length).
bool path_blocked(const Costmap& cm, const Path& p, uint8_t obs) {
    // Skip waypoint 0 (the start / robot cell): A* now lets the robot start in a
    // lethal cell (own-body noise / inflation), so flagging it would replan every
    // cycle for nothing. A lethal cell further ALONG the path is a real block.
    for (size_t i = 1; i < p.waypoints.size(); ++i) {
        int ix, iy;
        if (!cm.world_to_cell(p.waypoints[i].first, p.waypoints[i].second, ix, iy))
            return true;   // fell outside the (recentred) map
        if (cm.at(ix, iy) >= obs) return true;
    }
    return false;
}

// Peak (max) cell cost along the path — how close it comes to an obstacle. The
// backstop uses this to reroute once the committed path has degraded into a
// high-cost zone (people encroached / obstacle grew), before it's fully blocked.
float path_peak_cost(const Costmap& cm, const Path& p) {
    float peak = 0.0f;
    for (const auto& [wx, wy] : p.waypoints) {
        int ix, iy;
        if (cm.world_to_cell(wx, wy, ix, iy)) peak = std::max(peak, float(cm.at(ix, iy)));
    }
    return peak;
}

// Does the path pass through an object's footprint (its bbox grown by the robot
// radius clr)? Waypoints are ~1 cell apart, far finer than clr, so sampling the
// waypoints never skips over the box. Used to report what is blocking the path.
bool path_hits_object(const Path& p, const DetectedObject& o, float clr) {
    const float x0 = o.min_x - clr, x1 = o.max_x + clr;
    const float y0 = o.min_y - clr, y1 = o.max_y + clr;
    for (const auto& [wx, wy] : p.waypoints)
        if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) return true;
    return false;
}

// Robot farther than tol (m) from every waypoint -> it has left the path
// corridor (localization jump / pushed off course) and should replan.
bool robot_off_path(const Path& p, float rx, float ry, float tol) {
    if (p.waypoints.empty()) return false;
    float best2 = std::numeric_limits<float>::max();
    for (const auto& [wx, wy] : p.waypoints) {
        const float dx = wx - rx, dy = wy - ry;
        best2 = std::min(best2, dx * dx + dy * dy);
    }
    return best2 > tol * tol;
}

}  // namespace

bool Planner::start(DataBuffer<OccupancyGrid>& grid_src,
                    DataBuffer<ObjectList>&    objects_src,
                    const PlannerConfig&       cfg) {
    if (running_) return true;
    grid_src_    = &grid_src;
    objects_src_ = &objects_src;
    cfg_         = cfg;
    astar_       = AStarPlanner(cfg.astar);
    running_ = true;
    thread_  = std::thread(&Planner::run, this);
    return true;
}

void Planner::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void Planner::set_goal(float x, float y) {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    has_goal_ = true;
    goal_x_ = x;
    goal_y_ = y;
    ++goal_epoch_;
}

void Planner::clear_goal() {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    has_goal_ = false;
}

void Planner::run() {
    pthread_setname_np(pthread_self(), "planner");
    using clock = std::chrono::steady_clock;

    int64_t  last_grid   = -1;
    Path     cur;                       // committed path, kept while valid
    uint64_t planned_epoch = 0;         // goal epoch the committed path was planned for
    auto     last_replan = clock::now();
    const auto obs = uint8_t(cfg_.astar.obs_cost);

    while (running_) {
        auto grid = grid_src_->GetData();
        if (!grid || grid->empty() || grid->stamp_ns == last_grid) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        last_grid = grid->stamp_ns;

        // Costmap every frame (cheap EDT) — needed for the validity check AND so
        // the cost view stays live even with no goal. People footprints excluded.
        auto          objs = objects_src_->GetData();
        const cv::Mat dyn  = objs ? objs->dynamic_mask : cv::Mat();

        PlanResult out;
        out.stamp_ns = grid->stamp_ns;
        out.costmap  = cm_builder_.build(*grid, cfg_.grid, cfg_.costmap, dyn);

        float    gx, gy;
        bool     has;
        uint64_t epoch;
        { std::lock_guard<std::mutex> lk(goal_mtx_); has = has_goal_; gx = goal_x_; gy = goal_y_; epoch = goal_epoch_; }

        if (!has) {
            cur = Path{};                     // no goal -> drop the path
            out.has_goal = false;
            result_.SetData(std::move(out));
            processed_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        const float rx = grid->robot_x, ry = grid->robot_y;
        const std::pair<float, float> start{rx, ry}, goal{gx, gy};

        // Hard triggers: replan now, adopt the result unconditionally. A new goal,
        // a blocked path, or the robot leaving the corridor all invalidate `cur`.
        const bool goal_changed = (epoch != planned_epoch);
        const bool blocked  = !cur.waypoints.empty() && path_blocked(out.costmap, cur, obs);
        const bool deviated = !cur.waypoints.empty() &&
                              robot_off_path(cur, rx, ry, cfg_.replan_deviation_m);

        if (goal_changed || blocked || deviated) {
            cur           = astar_.plan(out.costmap, start, goal);
            planned_epoch = epoch;
            last_replan   = clock::now();
        } else {
            // Backstop: the cheap peak-cost check runs EVERY cycle; A* reruns only
            // when the path has degraded (peak cell cost >= threshold) or there's
            // no path yet — grazing an obstacle before it's fully blocked. The
            // replan is optimal, so it never comes out worse. min_interval only
            // throttles A* while the path stays degraded (a stuck path can't hammer
            // the search) — it does NOT delay reacting to a path that just degraded.
            const bool degraded = cur.waypoints.empty() ||
                                  path_peak_cost(out.costmap, cur) >= cfg_.replan_cost_threshold;
            const bool interval_ok =
                std::chrono::duration<float>(clock::now() - last_replan).count()
                    >= kBackstopMinInterval_s;
            if (degraded && interval_ok) {
                cur         = astar_.plan(out.costmap, start, goal);
                last_replan = clock::now();
            }
        }

        out.has_goal = true;
        out.path     = cur;

        // Report which known objects the committed path crosses (bbox + robot
        // radius). Runs every cycle — cheap O(objects x path len) — so a person
        // stepping onto the path is caught immediately, without a replan.
        if (!cur.waypoints.empty() && objs) {
            const float clr = cfg_.costmap.lethal_radius_m;
            for (const auto& o : objs->objects)
                if (path_hits_object(cur, o, clr))
                    out.blocking.push_back(o);
        }

        result_.SetData(std::move(out));
        processed_.fetch_add(1, std::memory_order_relaxed);
    }
}

} // namespace kist
