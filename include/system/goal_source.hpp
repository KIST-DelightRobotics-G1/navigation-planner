#pragma once

// GoalSource — the freshest active goal across the two input channels: rviz "2D Goal Pose"
// (ad-hoc, dock off) and named goal commands (catalog + dock). Whichever DataBuffer was set
// most recently wins. Consumers (planner + controller workers) share one GoalSource and must
// check Goal::valid — a cancel command lands as a fresh valid=false goal (= stop).

#include "common/data_buffer.hpp"
#include "goal_generation/goal.hpp"

#include <optional>

namespace kist {

class GoalSource {
public:
    void bind(DataBuffer<Goal>* rviz, DataBuffer<Goal>* named) { rviz_ = rviz; named_ = named; }

    std::optional<Goal> active() const {
        if (!rviz_ || !named_) return std::nullopt;
        auto a = rviz_->GetDataWithTime();
        auto b = named_->GetDataWithTime();
        if (a.HasData() && b.HasData()) return (a.GetAgeMs() <= b.GetAgeMs()) ? *a.data : *b.data;
        if (a.HasData()) return *a.data;
        if (b.HasData()) return *b.data;
        return std::nullopt;
    }

private:
    DataBuffer<Goal>* rviz_  = nullptr;
    DataBuffer<Goal>* named_ = nullptr;
};

} // namespace kist
