#pragma once

// GoalSource — the single active goal for the planner + controller, IN THE ODOM FRAME. It picks
// the freshest of the two input channels (rviz ad-hoc `rt/goal_pose`, odom; and named command
// `rt/kist/nav/goal`, map frame + dock), then converts a map-frame goal into odom using the
// latest map->odom estimate — every call, so a fixed map destination tracks despite LIO drift /
// relocalization jumps. Consumers just call active() and get an odom goal; check Goal::valid
// (a cancel, or a map goal with no localization yet, comes back valid=false = stop).

#include "common/data_buffer.hpp"
#include "goal_generation/goal.hpp"
#include "localization/map_odom.hpp"

#include <optional>

namespace kist {

class GoalSource {
public:
    void bind(DataBuffer<Goal>* rviz, DataBuffer<Goal>* named, DataBuffer<MapOdom>* mapodom) {
        rviz_ = rviz; named_ = named; mapodom_ = mapodom;
    }

    std::optional<Goal> active() const {
        std::optional<Goal> g;
        if (rviz_ && named_) {
            auto a = rviz_->GetDataWithTime();
            auto b = named_->GetDataWithTime();
            if (a.HasData() && b.HasData()) g = (a.GetAgeMs() <= b.GetAgeMs()) ? *a.data : *b.data;
            else if (a.HasData()) g = *a.data;
            else if (b.HasData()) g = *b.data;
        }
        if (!g) return std::nullopt;
        if (!g->in_map) return g;                         // odom goal (rviz) -> pass through

        // map-frame goal: needs the current map->odom to convert; none yet -> hold (stop).
        auto mo = mapodom_ ? mapodom_->GetData() : nullptr;
        if (!mo) { Goal ng = *g; ng.valid = false; return ng; }
        return goal_to_odom(*g, *mo);
    }

private:
    DataBuffer<Goal>*    rviz_    = nullptr;
    DataBuffer<Goal>*    named_   = nullptr;
    DataBuffer<MapOdom>* mapodom_ = nullptr;
};

} // namespace kist
