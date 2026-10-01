#pragma once

#include <string>
#include <vector>

namespace kist {

// The trace row layout, in order. Header-only so a probe/test can check it without the DDS/PCL
// includes. Keep in step with NavTrace::fill(). Enum-coded columns:
//   follow_phase   : FollowPhase   (0 Driving 1 Aligning 2 Approaching 3 Arrived 4 Blocked 5 NoPath 6 Idle)
//   subtask_status : SubtaskStatus (0 Idle 1 Running 2 Done 3 Failed)
//   loc_phase      : LocSample::Phase (0 search 1 track 2 lock-init 3 lock-fixed 4 uwb-spike 5 lost)
inline std::vector<std::string> nav_trace_columns() {
    std::vector<std::string> c;
    auto add  = [&](const std::string& n) { c.push_back(n); };
    auto addn = [&](const std::string& n, int k) { for (int i = 0; i < k; ++i) add(n + "_" + std::to_string(i)); };

    // tick
    add("t_s"); add("tick"); add("tick_us"); add("trace_dropped");
    // controller / follower (this tick, from the control step)
    add("follow_phase"); add("subtask_status"); add("progress");
    add("cmd_vx"); add("cmd_vy"); add("cmd_vyaw"); add("drive_enabled"); add("sent"); add("at_goal");
    add("arrival_hold"); add("nopath_hold");
    // active goal (leveled odom frame)
    add("goal_valid"); add("goal_in_map"); add("goal_x"); add("goal_y"); add("goal_yaw");
    add("goal_dist"); add("initial_dist"); add("dock_align"); add("dock_approach"); add("dock_standoff");
    add("subtask_index");
    // raw pose (odom / camera_init) — pelvis (sway-removed) + lidar (wobbles)
    add("pose_valid");
    add("opel_x"); add("opel_y"); add("opel_z"); add("opel_roll"); add("opel_pitch"); add("opel_yaw");
    add("olid_x"); add("olid_y"); add("olid_z");
    // gravity-leveled pose (planner frame) + floor tilt
    add("lev_valid"); add("lpel_x"); add("lpel_y"); add("lpel_yaw"); add("lev_tilt_deg");
    // planner / costmap
    add("path_valid"); add("path_n"); add("path_len_m"); add("path_age_ms");
    add("start_x"); add("start_y"); add("costmap_ready");
    // localization (EKF / GICP / UWB) — the diagnostics snapshot (~1 Hz, repeated between updates)
    add("loc_valid"); add("loc_phase"); add("mapodom_x"); add("mapodom_y"); add("mapodom_yaw");
    add("gicp_fitness"); add("submap_pts"); add("have_uwb"); add("uwb_map_x"); add("uwb_map_y");
    // pelvis IMU
    add("imu_valid"); addn("imu_quat", 4); addn("imu_gyro", 3);
    return c;
}

} // namespace kist
