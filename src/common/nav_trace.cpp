#include "common/nav_trace.hpp"
#include "common/nav_trace_columns.hpp"

#include "unitree/unitree_state_reader.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace kist {

namespace {
double quat_yaw(const Eigen::Quaterniond& q) {
    return std::atan2(2 * (q.w()*q.z() + q.x()*q.y()), 1 - 2 * (q.y()*q.y() + q.z()*q.z()));
}
void quat_rpy(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
    const double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    roll  = std::atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y));
    pitch = std::asin(std::clamp(2 * (w*y - z*x), -1.0, 1.0));
    yaw   = std::atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
}
}  // namespace

const std::vector<std::string>& NavTrace::columns() {
    static const std::vector<std::string> cols = nav_trace_columns();
    return cols;
}
size_t NavTrace::num_columns() { return columns().size(); }

NavTrace& NavTrace::instance() {
    static NavTrace inst;
    return inst;
}

bool NavTrace::start(const std::string& path) {
    if (enabled_) return true;
    std::error_code ec;
    auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent, ec);

    file_ = std::fopen(path.c_str(), "wb");
    if (!file_) {
        std::cerr << "[NavTrace] cannot open " << path << " — trace disabled\n";
        return false;
    }
    // Text header, then raw little-endian float32 rows.
    std::fprintf(file_, "NAVTRACE v1\ncolumns=%zu\nrate_hz=20\n", num_columns());
    const auto& cols = columns();
    for (size_t i = 0; i < cols.size(); ++i)
        std::fprintf(file_, "%s%s", i ? "," : "", cols[i].c_str());
    std::fprintf(file_, "\nEND\n");
    std::fflush(file_);

    ring_.assign(static_cast<size_t>(kRing) * num_columns(), 0.0f);
    head_ = tail_ = dropped_ = 0;
    tick_ = 0;
    start_time_ = std::chrono::steady_clock::now();
    stop_    = false;
    enabled_ = true;
    writer_  = std::thread(&NavTrace::writer_loop, this);
    std::cout << "[NavTrace] recording " << num_columns() << " columns/tick to " << path << "\n";
    return true;
}

void NavTrace::stop() {
    if (!enabled_) return;
    enabled_ = false;
    stop_    = true;
    if (writer_.joinable()) writer_.join();
    if (file_) { std::fclose(file_); file_ = nullptr; }
    std::cout << "[NavTrace] stopped (" << tick_ << " ticks, " << dropped_.load() << " dropped)\n";
}

void NavTrace::writer_loop() {
    const size_t ncol = num_columns();
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        uint32_t head = head_.load(std::memory_order_acquire);
        uint32_t tail = tail_.load(std::memory_order_relaxed);
        while (tail != head) {
            uint32_t idx = tail % kRing;
            uint32_t n   = std::min<uint32_t>(head - tail, kRing - idx);
            std::fwrite(ring_.data() + static_cast<size_t>(idx) * ncol, sizeof(float),
                        static_cast<size_t>(n) * ncol, file_);
            tail += n;
        }
        tail_.store(tail, std::memory_order_release);
        std::fflush(file_);
        if (stop_ && head_.load(std::memory_order_acquire) == tail) break;
    }
}

void NavTrace::record(const TickInfo& info) {
    if (!enabled_) return;
    uint32_t head = head_.load(std::memory_order_relaxed);
    uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= static_cast<uint32_t>(kRing)) {
        // writer behind: drop this row and count it — never wait, never touch tail_ (writer owns it)
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    float* row = ring_.data() + static_cast<size_t>(head % kRing) * num_columns();
    fill(info, row);
    head_.store(head + 1, std::memory_order_release);
}

void NavTrace::fill(const TickInfo& info, float* row) {
    const size_t ncol = num_columns();
    std::fill(row, row + ncol, 0.0f);
    size_t i = 0;
    auto put  = [&](double v) { row[i++] = static_cast<float>(v); };
    auto skip = [&](int k) { i += static_cast<size_t>(k); };

    // tick
    put(std::chrono::duration<double>(info.t0 - start_time_).count());
    put(static_cast<double>(tick_++));
    put(static_cast<double>(info.tick_us));
    put(static_cast<double>(dropped_.load(std::memory_order_relaxed)));

    // controller / follower
    put(info.follow_phase); put(info.subtask_status); put(info.progress);
    put(info.cmd_vx); put(info.cmd_vy); put(info.cmd_vyaw);
    put(info.drive_enabled); put(info.sent); put(info.at_goal);
    put(info.arrival_hold); put(info.nopath_hold);

    // goal
    put(info.goal_valid); put(info.goal_in_map); put(info.goal_x); put(info.goal_y); put(info.goal_yaw);
    put(info.goal_dist); put(info.initial_dist);
    put(info.dock_align); put(info.dock_approach); put(info.dock_standoff); put(info.subtask_index);

    // raw pose (odom)
    auto rt = pose_ ? pose_->GetData() : nullptr;
    if (rt) {
        const auto& p = rt->T_odom_pelvis.translation;
        double r, pit, yw; quat_rpy(rt->T_odom_pelvis.rotation, r, pit, yw);
        const auto& l = rt->T_odom_lidar.translation;
        put(1);
        put(p.x()); put(p.y()); put(p.z()); put(r); put(pit); put(yw);
        put(l.x()); put(l.y()); put(l.z());
    } else {
        put(0); skip(9);
    }

    // gravity-leveled pose + floor tilt
    auto lev = leveled_ ? leveled_->GetData() : nullptr;
    if (lev) {
        put(1);
        if (rt) {
            const Eigen::Vector3d lp = lev->transformPoint(rt->T_odom_pelvis.translation);
            const double lyaw = quat_yaw(lev->rotation * rt->T_odom_pelvis.rotation);
            put(lp.x()); put(lp.y()); put(lyaw);
        } else {
            skip(3);
        }
        const Eigen::Matrix3d R = lev->rotation.toRotationMatrix();
        put(std::acos(std::clamp(R(2, 2), -1.0, 1.0)) * 57.29577951308232);   // floor tilt (deg)
    } else {
        put(0); skip(4);
    }

    // planner / costmap
    auto pathT = path_ ? path_->GetDataWithTime() : TimestampedData<Path>{};
    if (pathT.HasData()) {
        const Path& p = *pathT.data;
        double len = 0.0;
        for (size_t k = 1; k < p.waypoints.size(); ++k)
            len += std::hypot(p.waypoints[k].first - p.waypoints[k-1].first,
                              p.waypoints[k].second - p.waypoints[k-1].second);
        put(1); put(static_cast<double>(p.waypoints.size())); put(len); put(pathT.GetAgeMs());
    } else {
        put(0); skip(3);
    }
    auto cm = costmap_ ? costmap_->GetData() : nullptr;
    if (cm) { put(cm->robot_x); put(cm->robot_y); put(cm->empty() ? 0 : 1); }
    else    { skip(2); put(0); }

    // localization snapshot (EKF / GICP / UWB)
    auto ls = loc_ ? loc_->GetData() : nullptr;
    if (ls) {
        put(1); put(ls->phase); put(ls->mapodom_x); put(ls->mapodom_y); put(ls->mapodom_yaw);
        put(ls->gicp_fitness); put(ls->submap_pts); put(ls->have_uwb); put(ls->uwb_map_x); put(ls->uwb_map_y);
    } else {
        put(0); skip(9);
    }

    // pelvis IMU
    if (auto st = UnitreeStateReader::instance().state_buf.GetData()) {
        put(1);
        for (double v : st->imu_pelvis.quaternion) put(v);
        for (double v : st->imu_pelvis.gyroscope)  put(v);
    } else {
        put(0); skip(4 + 3);
    }

    if (i != ncol)
        std::cerr << "[NavTrace] row layout mismatch: filled " << i << " of " << ncol << "\n";
}

} // namespace kist
