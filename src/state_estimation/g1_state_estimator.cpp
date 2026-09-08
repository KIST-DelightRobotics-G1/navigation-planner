#include "state_estimation/g1_state_estimator.hpp"

#include "kinematics/g1_kinematics.hpp"

#include "InEKF.h"          // inekf::InEKF, Kinematics, vectorKinematics
#include "RobotState.h"
#include "NoiseParams.h"

#include <Eigen/Dense>

#include <utility>
#include <vector>

namespace kist {

namespace {
LegJoints leftLeg(const LegJointSample& j) {
    LegJoints q;
    q.hip_pitch = j.left_hip_pitch; q.hip_roll = j.left_hip_roll; q.hip_yaw = j.left_hip_yaw;
    q.knee = j.left_knee; q.ankle_pitch = j.left_ankle_pitch; q.ankle_roll = j.left_ankle_roll;
    return q;
}
LegJoints rightLeg(const LegJointSample& j) {
    LegJoints q;
    q.hip_pitch = j.right_hip_pitch; q.hip_roll = j.right_hip_roll; q.hip_yaw = j.right_hip_yaw;
    q.knee = j.right_knee; q.ankle_pitch = j.right_ankle_pitch; q.ankle_roll = j.right_ankle_roll;
    return q;
}
Eigen::Matrix4d homogeneous(const Transform& T) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.block<3, 3>(0, 0) = T.rotation.toRotationMatrix();
    M.block<3, 1>(0, 3) = T.translation;
    return M;
}
inekf::NoiseParams makeNoise(const EstimatorConfig& c) {
    inekf::NoiseParams n;
    n.setGyroscopeNoise(c.gyro_noise);
    n.setAccelerometerNoise(c.accel_noise);
    n.setGyroscopeBiasNoise(c.gyro_bias_noise);
    n.setAccelerometerBiasNoise(c.accel_bias_noise);
    n.setContactNoise(c.contact_noise);
    return n;
}
}  // namespace

struct G1StateEstimator::Impl {
    EstimatorConfig cfg;
    inekf::InEKF    filter;
    EstimatorState  latest;

    bool                      have_prev_imu = false;
    int64_t                   prev_stamp_ns = 0;
    Eigen::Matrix<double, 6, 1> prev_imu = Eigen::Matrix<double, 6, 1>::Zero();

    explicit Impl(const EstimatorConfig& c)
        : cfg(c), filter(inekf::RobotState(), makeNoise(c)) {   // R=I, v=0, p=0
        refreshLatest(0);
    }

    void refreshLatest(int64_t stamp_ns) {
        inekf::RobotState s = filter.getState();     // library getters are non-const
        latest.pose.stamp_ns    = stamp_ns;
        latest.pose.position    = s.getPosition();
        latest.pose.orientation = Eigen::Quaterniond(s.getRotation());
        latest.linear_velocity  = s.getVelocity();
        latest.gyro_bias        = s.getGyroscopeBias();
        latest.accel_bias       = s.getAccelerometerBias();
    }
};

G1StateEstimator::G1StateEstimator(const EstimatorConfig& cfg)
    : impl_(std::make_unique<Impl>(cfg)) {}

G1StateEstimator::~G1StateEstimator() = default;

void G1StateEstimator::reset(const EstimatorState& init) {
    inekf::RobotState s;
    s.setRotation(init.pose.orientation.toRotationMatrix());
    s.setVelocity(init.linear_velocity);
    s.setPosition(init.pose.position);
    s.setGyroscopeBias(init.gyro_bias);
    s.setAccelerometerBias(init.accel_bias);
    impl_->filter.setState(s);
    impl_->have_prev_imu = false;
    impl_->refreshLatest(init.pose.stamp_ns);
}

void G1StateEstimator::propagate(const ImuSample& imu) {
    Eigen::Matrix<double, 6, 1> m;
    m << imu.gyro, imu.accel;

    if (impl_->have_prev_imu) {
        const double dt = double(imu.stamp_ns - impl_->prev_stamp_ns) * 1e-9;
        if (dt > 0.0 && dt <= impl_->cfg.max_propagate_dt_s)
            impl_->filter.Propagate(impl_->prev_imu, dt);   // integrate prev over dt
    }
    impl_->prev_imu      = m;
    impl_->prev_stamp_ns = imu.stamp_ns;
    impl_->have_prev_imu = true;
    impl_->refreshLatest(imu.stamp_ns);
}

void G1StateEstimator::updateKinematics(const LegJointSample& joints,
                                        const FootContactSample& contacts) {
    std::vector<std::pair<int, bool>> c;
    c.emplace_back(0, contacts.left_contact);    // id 0 = left foot
    c.emplace_back(1, contacts.right_contact);   // id 1 = right foot
    impl_->filter.setContacts(c);

    Eigen::Matrix<double, 6, 1> diag;
    const double r2 = impl_->cfg.fk_rotation_std * impl_->cfg.fk_rotation_std;
    const double p2 = impl_->cfg.fk_position_std * impl_->cfg.fk_position_std;
    diag << r2, r2, r2, p2, p2, p2;
    const Eigen::Matrix<double, 6, 6> cov = diag.asDiagonal();

    const Transform TL = G1Kinematics::pelvisToLeftFoot(leftLeg(joints));
    const Transform TR = G1Kinematics::pelvisToRightFoot(rightLeg(joints));

    inekf::vectorKinematics measured;
    measured.push_back(inekf::Kinematics(0, homogeneous(TL), cov));
    measured.push_back(inekf::Kinematics(1, homogeneous(TR), cov));
    impl_->filter.CorrectKinematics(measured);

    impl_->refreshLatest(joints.stamp_ns);
}

EstimatorState G1StateEstimator::state() const { return impl_->latest; }

} // namespace kist
