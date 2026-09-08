#include "occupancy_grid/occupancy_grid_builder.hpp"

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace kist {

bool OccupancyGridBuilder::start(DataBuffer<UnitreePointCloud>& lidar_src,
                                 DataBuffer<LabeledCloud>&      cloud_src,
                                 DataBuffer<CalibratedPose>&    pose_src,
                                 DataBuffer<UnitreeOdometry>&   odom_src,
                                 DataBuffer<WaistJoints>&       waist_src,
                                 const GridConfig& cfg,
                                 const ScanMatchConfig& sm_cfg) {
    if (running_) return true;
    lidar_src_ = &lidar_src;
    cloud_src_ = &cloud_src;
    pose_src_  = &pose_src;
    odom_src_  = &odom_src;
    waist_src_ = &waist_src;
    cfg_       = cfg;
    sm_cfg_    = sm_cfg;
    running_ = true;
    thread_  = std::thread(&OccupancyGridBuilder::run, this);
    return true;
}

void OccupancyGridBuilder::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void OccupancyGridBuilder::run() {
    pthread_setname_np(pthread_self(), "occ-grid");
    int64_t last_lidar = -1, last_cloud = -1;
    bool    world_anchored = false;

    while (running_) {
        auto lidar = lidar_src_->GetData();
        auto cloud = cloud_src_->GetData();
        auto pose  = pose_src_->GetData();
        const bool new_lidar = lidar && lidar->stamp_ns != last_lidar && lidar->point_count() > 0;
        const bool new_cloud = cloud && cloud->stamp_ns != last_cloud && !cloud->empty();
        if (!new_lidar && !new_cloud) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (new_lidar) last_lidar = lidar->stamp_ns;
        if (new_cloud) last_cloud = cloud->stamp_ns;

        // Tilt leveling from the latest base orientation (odom): cancel the mount
        // roll/pitch so the floor stays out of the obstacle band while the robot
        // bobs. Identity when disabled or no odom yet (assumes a level base).
        // Tilt leveling: prefer the RANSAC floor fit carried on the camera cloud
        // (measures the true sensor tilt), fall back to the base odom roll/pitch
        // (under-reports the head bob), else identity. Applied to BOTH clouds so
        // the floor stays out of the obstacle band as the mount bobs.
        Leveling lv;
        auto waist = cfg_.use_waist_fk ? waist_src_->GetData() : nullptr;
        if (waist && waist->valid) {
            // Waist FK: exact torso bob (rotation + translation) relative to the
            // standing reference. Delta = FK(now) * inv(FK(ref)) in the pelvis
            // frame, applied to the (fixed-extrinsics) robot-frame points.
            const Transform now = G1Kinematics::pelvisToTorso(waist->yaw, waist->roll, waist->pitch);
            if (!waist_ref_set_) { waist_ref_inv_ = now.inverse(); waist_ref_set_ = true; }
            const Transform d = now * waist_ref_inv_;
            const Eigen::Matrix3d R = d.rotation.toRotationMatrix();
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) lv.r[i*3+j] = float(R(i, j));
            lv.t[0] = float(d.translation.x());
            lv.t[1] = float(d.translation.y());
            lv.t[2] = float(d.translation.z());
            if (++dbg_ % 40 == 0)
                std::printf("[grid] waist FK: yaw=%.1f roll=%.1f pitch=%.1f deg  d_t=(%.3f %.3f %.3f)\n",
                            waist->yaw*57.2958f, waist->roll*57.2958f, waist->pitch*57.2958f,
                            d.translation.x(), d.translation.y(), d.translation.z());
        } else if (cloud && cloud->ground_valid) {
            lv = make_leveling_from_normal(cloud->ground_nx, cloud->ground_ny, cloud->ground_nz);
        } else if (cfg_.level_by_odom) {
            if (auto od = odom_src_->GetData()) {
                float roll, pitch;
                quat_to_roll_pitch(od->qx, od->qy, od->qz, od->qw, roll, pitch);
                lv = make_leveling(roll, pitch);
            }
        }

        Pose2D p;
        if (pose) {
            // Trusted pose -> world-anchored ACCUMULATION. The grid persists, so
            // integrate only the newly-arrived cloud each cycle; the other
            // sensor's past evidence is still in the belief.
            p = { pose->x, pose->y, pose->yaw };
            // Scan-match the UWB pose against the accumulated grid to cancel the
            // walking head-bob (small window, so it stays tied to UWB globally).
            // Uses the leveled LiDAR 2D scan vs the PRIOR map (before this frame).
            if (sm_cfg_.enabled && world_anchored && new_lidar && lidar) {
                scan2d_.clear();
                const float* pts = lidar->xyz.data();
                const size_t np  = lidar->point_count();
                for (size_t i = 0; i < np; i += 4) {   // subsample for speed
                    float lx, ly, lz;
                    lv.apply(pts[3*i], pts[3*i+1], pts[3*i+2], lx, ly, lz);
                    if (lz < cfg_.min_z || lz > cfg_.max_z) continue;
                    scan2d_.push_back(lx);
                    scan2d_.push_back(ly);
                }
                p = scan_match(grid_, cfg_, scan2d_, p, sm_cfg_);
            }
            if (!world_anchored) { grid_reset(grid_, cfg_, p.x, p.y); world_anchored = true; }
            else { grid_.robot_x = p.x; grid_.robot_y = p.y; }
            grid_.robot_yaw = p.yaw;
            grid_decay(grid_, cfg_);
            if (new_lidar) grid_integrate_lidar(grid_, *lidar, p, lv, cfg_);
            if (new_cloud) grid_integrate_labeled(grid_, *cloud, p, lv, cfg_);
        } else {
            // No pose -> robot-centered MEMORYLESS snapshot. The grid is wiped
            // each cycle, so integrate BOTH latest clouds every time (else the
            // display flickers between LiDAR-only and camera-only frames).
            world_anchored = false;
            p = { 0.f, 0.f, 0.f };
            grid_reset(grid_, cfg_, 0.f, 0.f);
            if (lidar && lidar->point_count() > 0) grid_integrate_lidar(grid_, *lidar, p, lv, cfg_);
            if (cloud && !cloud->empty())          grid_integrate_labeled(grid_, *cloud, p, lv, cfg_);
        }

        // Current-frame instance-per-cell map (per-frame indices; used only to
        // split touching same-class objects this cycle — never accumulated).
        cv::Mat inst_map;
        if (cloud && !cloud->instance.empty()) {
            inst_map = cv::Mat(grid_.n, grid_.n, CV_32S, cv::Scalar(-1));
            const float cs = std::cos(p.yaw), sn = std::sin(p.yaw);
            const size_t n = cloud->size();
            for (size_t i = 0; i < n; ++i) {
                float rx, ry, rz;
                lv.apply(cloud->xyz[3*i], cloud->xyz[3*i+1], cloud->xyz[3*i+2], rx, ry, rz);
                if (rz < cfg_.min_z || rz > cfg_.max_z) continue;
                const float wx = p.x + cs*rx - sn*ry, wy = p.y + sn*rx + cs*ry;
                int ix, iy;
                if (grid_.world_to_cell(wx, wy, ix, iy))
                    inst_map.at<int>(iy, ix) = cloud->instance[i];
            }
        }

        grid_.stamp_ns = std::max(last_lidar, last_cloud);
        grid_buf_.SetData(grid_);
        objects_buf_.SetData(extract_clusters(grid_, cfg_, inst_map));  // cluster inline (+ instance split)
        processed_.fetch_add(1, std::memory_order_relaxed);
    }
}

} // namespace kist
