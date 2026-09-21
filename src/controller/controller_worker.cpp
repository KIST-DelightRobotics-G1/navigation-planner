#include "controller/controller_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace kist {

void ControllerWorker::start(DataBuffer<Path>& path_buf, LioTransformProducer& prod,
                             DataBuffer<Costmap>& costmap_buf, GoalSource& goals,
                             DataBuffer<NavCommand>& cmd_buf, NavCommandPublisher& pub,
                             SubtaskStatePublisher& status_pub, bool drive_enabled,
                             const FollowConfig& fc, double arrival_hold_s) {
    path_buf_ = &path_buf; prod_ = &prod; costmap_buf_ = &costmap_buf; goals_ = &goals;
    cmd_buf_ = &cmd_buf; pub_ = &pub; status_pub_ = &status_pub; drive_enabled_ = drive_enabled;
    arrival_hold_s_ = arrival_hold_s;
    ctrl_.set_config(fc);
    running_ = true;
    thread_ = std::thread(&ControllerWorker::run, this);
}

void ControllerWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

ControllerWorker::SubtaskReport ControllerWorker::step_status(const std::optional<Goal>& goal,
                                                             const RobotTransforms* rt,
                                                             FollowPhase phase, double dt) {
    SubtaskReport r;
    const bool have_goal = goal && goal->valid;

    float rx = 0.f, ry = 0.f; bool have_pose = false;
    if (rt) { rx = float(rt->T_odom_pelvis.translation.x());
              ry = float(rt->T_odom_pelvis.translation.y()); have_pose = true; }
    auto goal_dist = [&]() -> float {
        return (goal && have_pose) ? std::hypot(goal->x - rx, goal->y - ry) : 0.f;
    };

    if (have_goal) {
        holding_ = false;                                   // a live goal supersedes any prior hold
        const std::string& pid = goal->plan_id;
        const uint16_t     idx = goal->index;
        if (!have_cur_ || pid != cur_plan_ || idx != cur_index_) {   // new subtask
            cur_plan_ = pid; cur_index_ = idx; have_cur_ = true;
            initial_dist_ = std::max(0.05f, goal_dist());    // capture start distance (avoid /0)
            arrival_hold_ = 0.0;
        }
        r.plan_id = pid; r.index = idx; r.action = goal->action;
        r.progress = std::clamp(1.0f - goal_dist() / initial_dist_, 0.0f, 1.0f);

        switch (phase) {
            case FollowPhase::Arrived:
                arrival_hold_ += dt;                         // settling at the goal
                if (arrival_hold_ >= arrival_hold_s_) {      // debounced -> DONE
                    holding_ = true;
                    arr_plan_ = pid; arr_index_ = idx; arr_action_ = goal->action;
                    r.status = SubtaskStatus::Done; r.progress = 1.0f;
                } else {
                    r.status = SubtaskStatus::Running;       // held, not yet confirmed
                }
                break;
            case FollowPhase::NoPath:
                arrival_hold_ = 0.0;
                r.status = SubtaskStatus::Failed; r.note = "no path";
                break;
            default:                                          // Driving/Aligning/Approaching/Blocked
                arrival_hold_ = 0.0;
                r.status = SubtaskStatus::Running;
                break;
        }
        return r;
    }

    // ── no active goal ──
    arrival_hold_ = 0.0;
    const GoalDisposition disp = goal ? goal->disp : GoalDisposition::None;
    if (disp == GoalDisposition::Failed) {                   // a fresh command was rejected
        holding_ = false; have_cur_ = false;
        r.status = SubtaskStatus::Failed; r.note = goal->note;
        r.plan_id = goal->plan_id; r.index = goal->index; r.action = goal->action;
        return r;
    }
    if (disp == GoalDisposition::Cancelled) {                // cancel -> IDLE, "cancelled"
        holding_ = false; have_cur_ = false;
        r.status = SubtaskStatus::Idle; r.note = "cancelled";
        return r;                                            // plan_id/index/action empty (IDLE)
    }
    if (holding_) {                                          // consumed arrival -> DONE persists
        r.status = SubtaskStatus::Done; r.progress = 1.0f;
        r.plan_id = arr_plan_; r.index = arr_index_; r.action = arr_action_;
        return r;
    }
    have_cur_ = false;                                       // genuinely idle
    r.status = SubtaskStatus::Idle;
    return r;
}

void ControllerWorker::run() {
    auto last_step   = std::chrono::steady_clock::now();
    auto last_status = last_step - std::chrono::milliseconds(100);   // publish immediately
    auto last_print  = last_step;
    bool prev_holding = false;
    while (running_) {
        const auto t0 = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(t0 - last_step).count();
        last_step = t0;

        auto pathT = path_buf_->GetDataWithTime();
        auto rtT   = prod_->out_buf.GetDataWithTime();
        auto cm    = costmap_buf_->GetData();
        auto goal  = goals_->active();
        const Path*            path = (pathT.HasData() && pathT.GetAgeMs() < 500.0) ? pathT.data.get() : nullptr;
        const RobotTransforms* rt   = (rtT.HasData()  && rtT.GetAgeMs()  < 500.0) ? rtT.data.get()  : nullptr;

        FollowPhase phase = FollowPhase::Arrived;
        NavCommand  cmd   = ctrl_.step(path, rt, cm.get(), goal ? &*goal : nullptr, &phase);

        cmd_buf_->SetData(cmd);
        if (drive_enabled_) pub_->publish(cmd);              // Twist -> gearsonic (robot moves)

        // ── subtask state (rt/cortex/nav/state) ──
        const SubtaskReport rep = step_status(goal, rt, phase, dt);
        // On the confirmed-arrival edge, CONSUME the goal: planner then emits an empty path
        // (rviz clears) and the robot idles at zero velocity while state holds DONE.
        if (holding_ && !prev_holding) goals_->notify_arrived();
        prev_holding = holding_;

        if (t0 - last_status >= std::chrono::milliseconds(100)) {   // 10 Hz, always on
            status_pub_->publish(rep.plan_id, rep.index, rep.action, rep.status, rep.progress, rep.note);
            last_status = t0;
        }

        if (t0 - last_print >= std::chrono::milliseconds(500)) {
            last_print = t0;
            const char* ss = rep.status == SubtaskStatus::Running ? "RUNNING"
                           : rep.status == SubtaskStatus::Done    ? "DONE"
                           : rep.status == SubtaskStatus::Failed  ? "FAILED" : "IDLE";
            std::printf("[controller] vx=% .2f vy=% .2f vyaw=% .2f  %-7s prog=%.2f  %s\n",
                        cmd.vx, cmd.vy, cmd.vyaw, ss, rep.progress,
                        drive_enabled_ ? "SENT" : "(preview)");
        }
        std::this_thread::sleep_until(t0 + std::chrono::milliseconds(50));   // ~20 Hz
    }
}

} // namespace kist
