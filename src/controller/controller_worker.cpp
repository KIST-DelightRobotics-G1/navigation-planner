#include "controller/controller_worker.hpp"

#include <chrono>
#include <cstdio>

namespace kist {

void ControllerWorker::start(DataBuffer<Path>& path_buf, LioTransformProducer& prod,
                             DataBuffer<Costmap>& costmap_buf, GoalSource& goals,
                             DataBuffer<NavCommand>& cmd_buf, NavCommandPublisher& pub,
                             bool drive_enabled, const FollowConfig& fc) {
    path_buf_ = &path_buf; prod_ = &prod; costmap_buf_ = &costmap_buf; goals_ = &goals;
    cmd_buf_ = &cmd_buf; pub_ = &pub; drive_enabled_ = drive_enabled;
    ctrl_.set_config(fc);
    running_ = true;
    thread_ = std::thread(&ControllerWorker::run, this);
}

void ControllerWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void ControllerWorker::run() {
    auto last_print = std::chrono::steady_clock::now();
    while (running_) {
        const auto t0 = std::chrono::steady_clock::now();

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
