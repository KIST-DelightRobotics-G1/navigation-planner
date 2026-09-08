#include "kinematics/waist_producer.hpp"

namespace kist {

bool updateWaistTransform(TransformTree& tree, const WaistSample& sample) {
    const Transform T_pelvis_torso =
        G1Kinematics::pelvisToTorso(sample.yaw_rad, sample.roll_rad, sample.pitch_rad);

    return tree.updateTransform(FrameId::Pelvis, FrameId::Torso,
                                T_pelvis_torso, sample.stamp_ns);
}

} // namespace kist
