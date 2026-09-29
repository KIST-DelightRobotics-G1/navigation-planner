#pragma once

// GoalSource — the single active goal for the planner + controller, IN THE ODOM FRAME. It picks
// the freshest of the two input channels (rviz ad-hoc `rt/goal_pose`, odom; and the orchestrator
// subtask `rt/cortex/nav/cmd`, map frame + dock), then converts a map-frame goal into odom using the
// latest map->odom estimate — every call, so a fixed map destination tracks despite LIO drift /
// relocalization jumps. Consumers just call active() and get an odom goal; check Goal::valid
// (a cancel, or a map goal with no localization yet, comes back valid=false = stop).
//
// ARRIVAL CONSUME: once the controller confirms it has arrived (notify_arrived()), the active goal
// is "consumed" — active() reports it valid=false (idle) so the planner emits an empty path (rviz
// clears) and the robot holds at zero velocity. The goal is identified by its buffer WRITE TIME, so
// re-sending the SAME destination (a fresh sample, newer timestamp) re-arms it and the robot drives
// again; a different goal likewise re-arms. Nothing is cleared until a new sample or a new goal.

#include "common/data_buffer.hpp"
#include "goal_generation/goal.hpp"
#include "localization/map_odom.hpp"
#include "transforms/transform.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <mutex>
#include <optional>

namespace kist {

class GoalSource {
public:
    void bind(DataBuffer<Goal>* rviz, DataBuffer<Goal>* named, DataBuffer<MapOdom>* mapodom,
              DataBuffer<Transform>* leveled) {
        rviz_ = rviz; named_ = named; mapodom_ = mapodom; leveled_ = leveled;
    }

    std::optional<Goal> active() const {
        auto picked = pick_raw();
        if (!picked) return std::nullopt;

        {   // consumed (arrived) + still the same sample -> report idle (valid=false)
            std::lock_guard<std::mutex> lk(mtx_);
            if (consumed_ && picked->timestamp == consumed_ts_) {
                Goal ng = *picked->data; ng.valid = false; return ng;
            }
        }

        const Goal& g = *picked->data;
        if (!g.in_map) return level(g);                   // odom goal (rviz) -> level -> planner frame

        // map-frame goal: needs the current map->odom to convert; none yet -> hold (stop).
        auto mo = mapodom_ ? mapodom_->GetData() : nullptr;
        if (!mo) { Goal ng = g; ng.valid = false; return ng; }
        return level(goal_to_odom(g, *mo));               // odom -> leveled (same frame as the grid)
    }

    // The controller calls this once it has CONFIRMED arrival (debounced). It snapshots the active
    // goal's buffer write-time; active() then reports idle until a newer sample / different goal.
    void notify_arrived() {
        auto picked = pick_raw();
        if (!picked) return;
        std::lock_guard<std::mutex> lk(mtx_);
        consumed_ = true; consumed_ts_ = picked->timestamp;
    }

private:
    // Convert an ODOM goal into the gravity-leveled planner frame (identity until the leveler locks).
    // A goal is a floor point with no z, so project it onto the floor plane (recovered from
    // T_leveled_odom) before transforming, else robot and goal disagree by ~tilt*floor_height.
    Goal level(Goal g) const {
        if (!g.valid || !leveled_) return g;
        auto T = leveled_->GetData();
        if (!T) return g;
        const Eigen::Matrix3d R = T->rotation.toRotationMatrix();
        const Eigen::Vector3d n(R(2,0), R(2,1), R(2,2));  // floor normal (leveled +z) in odom
        const double n_dot_c = -T->translation.z();       // plane: n.p = n.c  (t.z = -n.c)
        const double gz = (std::abs(n.z()) > 1e-6)
            ? (n_dot_c - n.x()*g.x - n.y()*g.y) / n.z()   // floor z under the goal (odom)
            : 0.0;
        const Eigen::Vector3d gl = T->transformPoint(Eigen::Vector3d(g.x, g.y, gz));
        g.x = float(gl.x()); g.y = float(gl.y());          // yaw preserved by the leveling (heading)
        return g;
    }

    // The freshest of the two goal channels, with its buffer write-time (nullopt if neither has data).
    std::optional<TimestampedData<Goal>> pick_raw() const {
        if (!rviz_ || !named_) return std::nullopt;
        auto a = rviz_->GetDataWithTime();
        auto b = named_->GetDataWithTime();
        if (a.HasData() && b.HasData()) return (a.GetAgeMs() <= b.GetAgeMs()) ? a : b;
        if (a.HasData()) return a;
        if (b.HasData()) return b;
        return std::nullopt;
    }

    DataBuffer<Goal>*    rviz_    = nullptr;
    DataBuffer<Goal>*    named_   = nullptr;
    DataBuffer<MapOdom>* mapodom_ = nullptr;
    DataBuffer<Transform>* leveled_ = nullptr;   // T_leveled_odom (odom -> planner frame)

    mutable std::mutex                            mtx_;         // guards the consume latch
    bool                                          consumed_ = false;
    std::chrono::steady_clock::time_point         consumed_ts_{};
};

} // namespace kist
