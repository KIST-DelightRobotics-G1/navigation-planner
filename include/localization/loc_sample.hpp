#pragma once

#include <cstdint>

namespace kist {

// A lightweight per-cycle snapshot of the localization state, published by LocalizationWorker for
// the run trace (NavTrace) and any other observer. This is diagnostics only — the authoritative
// map->odom for the planner is MapOdom on mapodom_buf. Written ~1 Hz (the localization loop rate).
struct LocSample {
    enum Phase : int {   // matches the console phase strings
        Search        = 0,   // "search"        — accumulating, not yet locked
        Track         = 1,   // "track"         — EKF fusing GICP + UWB
        LockInit      = 2,   // "LOCK(init)"    — global_init just accepted
        LockFixedSeed = 3,   // "LOCK(fixed-seed)" — no UWB, fixed seed
        UwbSpike      = 4,   // "track(uwb-spike)" — a UWB update was gated out this cycle
        LostReinit    = 5,   // "LOST->reinit"  — sustained disagreement, lock dropped
    };

    int   phase       = Search;
    float mapodom_x   = 0.f, mapodom_y = 0.f, mapodom_yaw = 0.f;  // EKF T_map_odom (yaw rad)
    float gicp_fitness = -1.f;   // last GICP fitness (m^2); -1 if none this cycle
    int   submap_pts  = 0;
    int   have_uwb    = 0;
    float uwb_map_x   = 0.f, uwb_map_y = 0.f;   // UWB-derived robot base xy in map (valid if have_uwb)
};

} // namespace kist
