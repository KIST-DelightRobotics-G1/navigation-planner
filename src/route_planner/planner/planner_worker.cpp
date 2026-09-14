#include "route_planner/planner/planner_worker.hpp"

#include <chrono>

namespace kist {

void PlannerWorker::start(DataBuffer<Costmap>& costmap_buf, GoalSource& goals,
                          DataBuffer<Path>& path_buf) {
    costmap_buf_ = &costmap_buf; goals_ = &goals; path_buf_ = &path_buf;
    running_ = true;
    thread_ = std::thread(&PlannerWorker::run, this);
}

void PlannerWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void PlannerWorker::run() {
    while (running_) {
        auto goal = goals_->active();
        auto cm   = costmap_buf_->GetData();
        if (goal && goal->valid && cm && !cm->empty())
            path_buf_->SetData(planner_.plan(*cm, {cm->robot_x, cm->robot_y}, {goal->x, goal->y}));
        else
            path_buf_->SetData(Path{});   // no / cancelled goal -> empty path -> controller stops
        std::this_thread::sleep_for(std::chrono::milliseconds(200));   // ~5 Hz replan
    }
}

} // namespace kist
