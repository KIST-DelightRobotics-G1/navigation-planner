#include "controller/controller_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace kist {

void ControllerWorker::start(DataBuffer<Path>& path_buf, LioTransformProducer& prod,
                             DataBuffer<Costmap>& costmap_buf, GoalSource& goals,
                             DataBuffer<NavCommand>& cmd_buf, NavCommandPublisher& pub,
                             SubtaskStatePublisher& status_pub, DataBuffer<Transform>& leveled_buf,
                             bool drive_enabled, const FollowConfig& fc,
                             double arrival_hold_s, double nopath_hold_s) {
    path_buf_ = &path_buf; prod_ = &prod; costmap_buf_ = &costmap_buf; goals_ = &goals;
    cmd_buf_ = &cmd_buf; pub_ = &pub; status_pub_ = &status_pub; leveled_buf_ = &leveled_buf;
    drive_enabled_ = drive_enabled;
    arrival_hold_s_ = arrival_hold_s; nopath_hold_s_ = nopath_hold_s;
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

    // (A) A terminal verdict (DONE/FAILED) is emitted for exactly 3 publishes, then IDLE.
    if (term_active_) {
        r.status = term_status_; r.plan_id = term_plan_; r.index = term_index_;
        r.action = term_action_; r.detail = term_detail_;
        r.progress = (term_status_ == SubtaskStatus::Done) ? 1.0f : term_progress_;
        if (++term_count_ >= 3) {
            term_active_ = false;
            closed_plan_ = term_plan_; closed_index_ = term_index_; have_closed_ = true;
        }
        return r;
    }

    auto start_terminal = [&](SubtaskStatus st, const std::string& pid, uint16_t idx,
                              const std::string& act, const std::string& detail, float prog) {
        term_active_ = true; term_count_ = 1;
        term_status_ = st; term_plan_ = pid; term_index_ = idx; term_action_ = act;
        term_detail_ = detail; term_progress_ = prog;
        r.status = st; r.plan_id = pid; r.index = idx; r.action = act; r.detail = detail;
        r.progress = (st == SubtaskStatus::Done) ? 1.0f : prog;
    };

    const bool have_goal = goal && goal->valid;
    float rx = 0.f, ry = 0.f; bool have_pose = false;
    if (rt) { rx = float(rt->T_odom_pelvis.translation.x());
              ry = float(rt->T_odom_pelvis.translation.y()); have_pose = true; }
    auto goal_dist = [&]() -> float {
        return (goal && have_pose) ? std::hypot(goal->x - rx, goal->y - ry) : 0.f;
    };

    // (B) an active subtask
    if (have_goal) {
        const std::string& pid = goal->plan_id;
        const uint16_t     idx = goal->index;
        if (!have_cur_ || pid != cur_plan_ || idx != cur_index_) {   // new subtask
            cur_plan_ = pid; cur_index_ = idx; have_cur_ = true;
            initial_dist_ = std::max(0.05f, goal_dist());
            arrival_hold_ = 0.0; nopath_hold_ = 0.0; have_closed_ = false;
        }
        const float prog = std::clamp(1.0f - goal_dist() / initial_dist_, 0.0f, 1.0f);

        if (phase == FollowPhase::Arrived) {
            arrival_hold_ += dt; nopath_hold_ = 0.0;
            if (arrival_hold_ >= arrival_hold_s_) {         // debounced -> DONE
                goals_->notify_arrived();                   // drop the subtask (path clears, robot idles)
                start_terminal(SubtaskStatus::Done, pid, idx, goal->action, "", 1.0f);
                return r;
            }
        } else if (phase == FollowPhase::NoPath) {
            nopath_hold_ += dt; arrival_hold_ = 0.0;
            if (nopath_hold_ >= nopath_hold_s_) {           // sustained no-path -> FAILED
                goals_->notify_arrived();
                start_terminal(SubtaskStatus::Failed, pid, idx, goal->action, "no path", prog);
                return r;
            }
        } else {
            arrival_hold_ = 0.0; nopath_hold_ = 0.0;
        }
        r.status = SubtaskStatus::Running; r.plan_id = pid; r.index = idx;
        r.action = goal->action; r.progress = prog;
        return r;
    }

    // (C) no active goal
    arrival_hold_ = 0.0; nopath_hold_ = 0.0; have_cur_ = false;
    const GoalDisposition disp = goal ? goal->disp : GoalDisposition::None;
    if (disp == GoalDisposition::Failed) {                  // rejected command (unsupported / bad args)
        const bool already = have_closed_ && goal->plan_id == closed_plan_ && goal->index == closed_index_;
        if (!already) {
            start_terminal(SubtaskStatus::Failed, goal->plan_id, goal->index, goal->action, goal->note, 0.0f);
            return r;
        }
        r.status = SubtaskStatus::Idle;                     // already reported -> idle
        return r;
    }
    if (disp == GoalDisposition::Cancelled) {               // cancel -> IDLE, "cancelled"
        r.status = SubtaskStatus::Idle; r.detail = "cancelled";
        return r;
    }
    r.status = SubtaskStatus::Idle;                         // genuinely idle
    return r;
}

void ControllerWorker::run() {
    auto last_status = std::chrono::steady_clock::now() - std::chrono::milliseconds(100);
    auto last_print  = std::chrono::steady_clock::now();
    while (running_) {
        const auto t0 = std::chrono::steady_clock::now();

        auto pathT = path_buf_->GetDataWithTime();
        auto rtT   = prod_->out_buf.GetDataWithTime();
        auto cm    = costmap_buf_->GetData();
        auto goal  = goals_->active();
        const Path*            path = (pathT.HasData() && pathT.GetAgeMs() < 500.0) ? pathT.data.get() : nullptr;
        const RobotTransforms* rt   = (rtT.HasData()  && rtT.GetAgeMs()  < 500.0) ? rtT.data.get()  : nullptr;

        // Level the base pose into the planner frame (grid/goal/path are leveled). Identity until lock.
        RobotTransforms rt_lev;
        const RobotTransforms* rtp = nullptr;
        if (rt) {
            rt_lev = *rt;
            if (auto T = leveled_buf_ ? leveled_buf_->GetData() : nullptr) {
                rt_lev.T_odom_pelvis.translation =
                    T->rotation * rt->T_odom_pelvis.translation + T->translation;
                rt_lev.T_odom_pelvis.rotation = T->rotation * rt->T_odom_pelvis.rotation;
            }
            rtp = &rt_lev;
        }

        FollowPhase phase = FollowPhase::Arrived;
        NavCommand  cmd   = ctrl_.step(path, rtp, cm.get(), goal ? &*goal : nullptr, &phase);

        cmd_buf_->SetData(cmd);
        if (drive_enabled_) pub_->publish(cmd);              // Twist -> gearsonic (robot moves), ~20 Hz

        SubtaskReport rep;
        if (t0 - last_status >= std::chrono::milliseconds(100)) {   // status FSM + publish @ 10 Hz
            const double dt = std::chrono::duration<double>(t0 - last_status).count();
            last_status = t0;
            rep = step_status(goal, rtp, phase, dt);
            status_pub_->publish(rep.plan_id, rep.index, rep.action, rep.status, rep.progress, rep.detail);

            if (t0 - last_print >= std::chrono::milliseconds(500)) {
                last_print = t0;
                const char* ss = rep.status == SubtaskStatus::Running ? "RUNNING"
                               : rep.status == SubtaskStatus::Done    ? "DONE"
                               : rep.status == SubtaskStatus::Failed  ? "FAILED" : "IDLE";
                std::printf("[controller] vx=% .2f vy=% .2f vyaw=% .2f  %-7s prog=%.2f  %s\n",
                            cmd.vx, cmd.vy, cmd.vyaw, ss, rep.progress,
                            drive_enabled_ ? "SENT" : "(preview)");
            }
        }
        std::this_thread::sleep_until(t0 + std::chrono::milliseconds(50));   // ~20 Hz control
    }
}

} // namespace kist
