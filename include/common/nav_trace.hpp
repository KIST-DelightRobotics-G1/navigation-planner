#pragma once

// NavTrace — per-tick numeric trace of the whole nav path (controller + planner + pose + leveler +
// localization + IMU), so a run can be reconstructed afterwards instead of guessed at. One
// fixed-width float32 row per control tick (~20 Hz), written to logs/latest.trace next to the
// console mirror (overwritten each run). The file opens with a text header naming every column, so
// a reader needs nothing but the file (tools/read_trace.py). Ported structure from
// kist-gearsonic-inference (StateTrace), columns re-shaped for navigation.
//
// Cost model, so the control loop NEVER waits on it:
//  - the control thread fills a row and copies it into a preallocated ring — no allocation, no I/O;
//  - a low-priority thread drains the ring to disk every 200 ms;
//  - if the writer falls behind, new rows are dropped and counted (`trace_dropped`) — the control
//    thread is never blocked.
//
// The control thread supplies only what it alone knows (TickInfo); everything else (pose, leveled
// transform, path, costmap, localization snapshot, IMU) is pulled from the bound buffers inside
// record(), each a brief lock-free-ish DataBuffer read.

#include "common/data_buffer.hpp"
#include "localization/loc_sample.hpp"
#include "lio/robot_transforms.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/planner/astar_planner/path.hpp"
#include "transforms/transform.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace kist {

class NavTrace {
public:
    static const std::vector<std::string>& columns();
    static size_t num_columns();

    // What only the controller step knows this tick. The goal fields mirror the active Goal (leveled
    // odom frame); leave goal_valid=0 when there is none.
    struct TickInfo {
        std::chrono::steady_clock::time_point t0{};
        long  tick_us = 0;            // compute time of this control step
        int   follow_phase = 0;       // FollowPhase
        int   subtask_status = 0;     // SubtaskStatus
        float progress = 0.f;
        float cmd_vx = 0.f, cmd_vy = 0.f, cmd_vyaw = 0.f;
        int   drive_enabled = 0, sent = 0, at_goal = 0;
        float arrival_hold = 0.f, nopath_hold = 0.f;
        int   goal_valid = 0, goal_in_map = 0;
        float goal_x = 0.f, goal_y = 0.f, goal_yaw = 0.f;
        float goal_dist = 0.f, initial_dist = 0.f;
        int   dock_align = 0, dock_approach = 0;
        float dock_standoff = 0.f;
        int   subtask_index = 0;
    };

    static NavTrace& instance();

    // Bind the source buffers the trace reads from (set once, before start()). `pose` is normally
    // the LioTransformProducer's out_buf. Any may be null (those columns stay 0 / *_valid=0).
    void bind(DataBuffer<RobotTransforms>* pose, DataBuffer<Transform>* leveled,
              DataBuffer<Path>* path, DataBuffer<Costmap>* costmap, DataBuffer<LocSample>* loc) {
        pose_ = pose; leveled_ = leveled; path_ = path; costmap_ = costmap; loc_ = loc;
    }

    bool start(const std::string& path);
    void stop();
    bool enabled() const { return enabled_.load(); }

    // Control thread, once per tick. No-op when not started (a single atomic load).
    void record(const TickInfo& info);

private:
    NavTrace() = default;
    void writer_loop();
    void fill(const TickInfo& info, float* row);

    static constexpr int kRing = 512;           // ~25 s at 20 Hz
    std::vector<float>    ring_;                 // kRing * num_columns
    std::atomic<uint32_t> head_{0};             // rows produced
    std::atomic<uint32_t> tail_{0};             // rows written
    std::atomic<uint32_t> dropped_{0};
    uint64_t              tick_{0};
    std::chrono::steady_clock::time_point start_time_{};

    DataBuffer<RobotTransforms>* pose_    = nullptr;
    DataBuffer<Transform>*       leveled_ = nullptr;
    DataBuffer<Path>*            path_    = nullptr;
    DataBuffer<Costmap>*         costmap_ = nullptr;
    DataBuffer<LocSample>*       loc_     = nullptr;

    std::FILE*        file_{nullptr};
    std::thread       writer_;
    std::atomic<bool> enabled_{false};
    std::atomic<bool> stop_{false};
};

} // namespace kist
