#include "controller/controller_worker.hpp"

#include <chrono>
#include <cstdio>

namespace kist {

void ControllerWorker::start(DataBuffer<Path>& path_buf, LioTransformProducer& prod,
                             DataBuffer<Costmap>& costmap_buf, GoalSource& goals,
                             DataBuffer<NavCommand>& cmd_buf, NavCommandPublisher& pub,
                             NavStatusPublisher& status_pub, bool drive_enabled,
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

NavState ControllerWorker::resolve_state(bool have_goal, const std::string& name,
                                         FollowPhase phase, double dt) {
    if (have_goal) {
        holding_ = false;                               // a live goal supersedes any prior hold
        if (phase == FollowPhase::Arrived) {
            arrival_hold_ += dt;                        // settling at the goal
            if (arrival_hold_ >= arrival_hold_s_) {     // debounced -> confirm arrival
                holding_ = true; arrived_name_ = name;
                return NavState::Arrived;
            }
            return NavState::Approaching;               // held, not yet confirmed -> "arriving"
        }
        arrival_hold_ = 0.0;
        switch (phase) {
            case FollowPhase::Blocked:  return NavState::Blocked;
            case FollowPhase::Aligning: return NavState::Aligning;
            default:                    return phase == FollowPhase::Approaching ? NavState::Approaching
                                                                                 : NavState::Driving;
        }
    }
    // No live goal: either we consumed it on arrival (hold ARRIVED), or genuinely idle.
    arrival_hold_ = 0.0;
    if (holding_) return NavState::Arrived;             // persistent ARRIVED until a new goal
    arrived_name_.clear();
    return NavState::Idle;
}

void ControllerWorker::run() {
    auto last_print = std::chrono::steady_clock::now();
    auto last_step  = last_print;
    auto last_status = last_print - std::chrono::seconds(1);   // force an immediate first publish
    NavState last_state = NavState::Idle;
    bool     have_last_state = false;
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
        if (drive_enabled_) pub_->publish(cmd);          // Twist -> gearsonic (robot moves)

        // ── navigation status (rt/kist/nav/status): debounced state, on change + low-rate heartbeat ──
        const bool        have_goal = goal && goal->valid;
        const std::string gname     = have_goal ? goal->name : std::string();
        const NavState    state     = resolve_state(have_goal, gname, phase, dt);
        // Name to report: the live goal, or (while holding a consumed arrival) the goal we reached.
        const std::string pub_name  = have_goal ? gname : (holding_ ? arrived_name_ : std::string());
        // On the confirmed-arrival edge, CONSUME the goal: the planner then emits an empty path
        // (rviz clears) and the robot idles at zero velocity, while status holds ARRIVED (holding_).
        const bool arrived_edge = state == NavState::Arrived &&
                                  !(have_last_state && last_state == NavState::Arrived);
        if (!have_last_state || state != last_state ||
            t0 - last_status >= std::chrono::milliseconds(500)) {
            status_pub_->publish(pub_name, state);
            last_status = t0; last_state = state; have_last_state = true;
        }
        if (arrived_edge) goals_->notify_arrived();

        if (t0 - last_print >= std::chrono::milliseconds(500)) {
            last_print = t0;
            const char* ps = phase == FollowPhase::Blocked     ? "BLOCKED"
                           : phase == FollowPhase::Aligning    ? "align"
                           : phase == FollowPhase::Approaching ? "approach"
                           : phase == FollowPhase::Arrived     ? "arrived" : "drive";
            std::printf("[controller] vx=% .2f vy=% .2f vyaw=% .2f  %-7s  %s\n",
                        cmd.vx, cmd.vy, cmd.vyaw, ps, drive_enabled_ ? "SENT" : "(preview)");
        }
        std::this_thread::sleep_until(t0 + std::chrono::milliseconds(50));   // ~20 Hz
    }
}

} // namespace kist
