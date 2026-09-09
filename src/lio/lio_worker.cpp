#include "lio/lio_worker.hpp"

#include <omp.h>
#include "use-ikfom.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "lio/common_lib.h"
#include "lio/imu_processing.hpp"
#include "lio/cloud_converter.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cstring>
#include <memory>
#include <vector>

// Ported from FAST-LIO2 laserMapping.cpp: the ESIKF + ikd-Tree mapping core, with
// the ROS node / subscribers / publishers / PCD-save / logging stripped. h_share_model
// is the ESIKF measurement model (point-to-plane against the ikd-Tree map); it is a
// free function the filter calls by pointer, so the state below is file-scope. One
// LIO per process (see lio_worker.hpp).

#define INIT_TIME       (0.1)
#define LASER_POINT_COV (0.001)
#define MOV_THRESHOLD   (1.5f)

namespace {

// The map is leaked (never destructed): ~KD_TREE races its rebuild-thread teardown
// on shutdown; keeping it alive for the whole process avoids that.
KD_TREE<PointType>& ikdtree = *(new KD_TREE<PointType>());

esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3       pos_lid;

std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

std::vector<PointVector> Nearest_Points;
bool  point_selected_surf[100000] = {0};
float res_last[100000] = {0.0};

pcl::VoxelGrid<PointType> downSizeFilterSurf;

int    feats_down_size = 0;
int    effct_feat_num  = 0;
double total_residual  = 0.0;
double res_mean_last   = 0.05;
double filter_size_map_min = 0.5;
double cube_len   = 200.0;
float  DET_RANGE  = 100.0f;
bool   extrinsic_est_en = true;
bool   flg_EKF_inited   = false;
bool   flg_first_scan   = true;
double first_lidar_time = 0.0;

// lasermap_fov_segment state
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
std::vector<BoxPointType> cub_needrm;
V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
int    add_point_size = 0, kdtree_delete_counter = 0;
double match_time = 0, solve_time = 0, kdtree_incremental_time = 0, kdtree_delete_time = 0;

// ─── point transforms ─────────────────────────────────────────────────────────
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
    po->x = p_global(0); po->y = p_global(1); po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
    po[0] = p_global(0); po[1] = p_global(1); po[2] = p_global(2);
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
}

// ─── moving local map ─────────────────────────────────────────────────────────
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if (cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

// ─── add scan points to the map ───────────────────────────────────────────────
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++) {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            PointType mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) { need_add = false; break; }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
}

// ─── ESIKF measurement model: point-to-plane against the ikd-Tree map ─────────
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    for (int i = 0; i < feats_down_size; i++) {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0); point_world.y = p_global(1); point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge) {
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s2 = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            if (s2 > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) { ekfom_data.valid = false; return; }
    res_mean_last = total_residual / effct_feat_num;

    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en) {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

} // anonymous namespace

namespace kist {

struct LioWorker::Impl {
    LioConfig cfg;
};

LioWorker::LioWorker(const LioConfig& cfg) : impl_(std::make_unique<Impl>()) {
    impl_->cfg = cfg;

    filter_size_map_min = cfg.filter_size_map;
    cube_len            = cfg.cube_len;
    DET_RANGE           = float(cfg.det_range);
    downSizeFilterSurf.setLeafSize(cfg.filter_size_surf, cfg.filter_size_surf, cfg.filter_size_surf);
    ikdtree.set_downsample_param(cfg.filter_size_map);

    p_imu->set_extrinsic(V3D(cfg.ext_t), M3D(cfg.ext_R));
    p_imu->set_gyr_cov(V3D(cfg.gyr_cov, cfg.gyr_cov, cfg.gyr_cov));
    p_imu->set_acc_cov(V3D(cfg.acc_cov, cfg.acc_cov, cfg.acc_cov));
    p_imu->set_gyr_bias_cov(V3D(cfg.b_gyr_cov, cfg.b_gyr_cov, cfg.b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(cfg.b_acc_cov, cfg.b_acc_cov, cfg.b_acc_cov));

    double epsi[23];
    std::fill(epsi, epsi + 23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, cfg.max_iterations, epsi);

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
}

LioWorker::~LioWorker() = default;

LioPose LioWorker::process(const UnitreePointCloud& frame, const std::deque<ImuSample>& imu) {
    LioPose out;

    MeasureGroup meas;
    to_fastlio_cloud(frame, *meas.lidar, impl_->cfg.blind, impl_->cfg.point_filter_num);
    meas.lidar_beg_time = frame.stamp_ns * 1e-9;
    const double span_s = meas.lidar->points.empty() ? 0.0 : meas.lidar->points.back().curvature / 1000.0;
    meas.lidar_end_time = meas.lidar_beg_time + span_s;
    meas.imu = imu;
    if (meas.imu.empty()) return out;

    if (flg_first_scan) {
        first_lidar_time = meas.lidar_beg_time;
        p_imu->first_lidar_time = first_lidar_time;
        flg_first_scan = false;
        return out;
    }

    p_imu->Process(meas, kf, feats_undistort);
    state_point = kf.get_x();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    if (feats_undistort->empty()) return out;

    flg_EKF_inited = (meas.lidar_beg_time - first_lidar_time) >= INIT_TIME;
    lasermap_fov_segment();

    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);
    feats_down_size = feats_down_body->points.size();

    if (ikdtree.Root_Node == nullptr) {
        if (feats_down_size > 5) {
            ikdtree.set_downsample_param(filter_size_map_min);
            feats_down_world->resize(feats_down_size);
            for (int i = 0; i < feats_down_size; i++)
                pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            ikdtree.Build(feats_down_world->points);
        }
        return out;   // map still initializing
    }
    if (feats_down_size < 5) return out;

    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);
    Nearest_Points.resize(feats_down_size);

    double solve_H_time = 0;
    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    state_point = kf.get_x();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

    map_incremental();

    out.stamp_ns = frame.stamp_ns;
    out.position = pos_lid;
    const M3D R_ol = state_point.rot.toRotationMatrix() * state_point.offset_R_L_I.toRotationMatrix();
    out.orientation = Eigen::Quaterniond(R_ol);
    out.linear_velocity = state_point.vel;
    out.valid = flg_EKF_inited;
    return out;
}

} // namespace kist
