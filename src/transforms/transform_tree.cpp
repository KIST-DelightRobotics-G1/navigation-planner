#include "transforms/transform_tree.hpp"

#include <vector>

namespace kist {

bool TransformTree::setStaticTransform(FrameId parent, FrameId child,
                                       const Transform& T) {
    // Topology is fixed by frame_parent(); the parent arg is a consistency guard.
    if (frame_parent(child) != std::optional<FrameId>(parent)) return false;
    static_edges_[static_cast<std::size_t>(child)] = T;
    return true;
}

bool TransformTree::updateTransform(FrameId parent, FrameId child,
                                    const Transform& T, int64_t stamp_ns) {
    if (frame_parent(child) != std::optional<FrameId>(parent)) return false;
    dynamic_edges_[static_cast<int>(child)].push(stamp_ns, T);
    return true;
}

std::optional<Transform> TransformTree::edgeTransform(FrameId child,
                                                      int64_t stamp_ns) const {
    if (const auto& s = static_edges_[static_cast<std::size_t>(child)]) return s;
    const auto it = dynamic_edges_.find(static_cast<int>(child));
    if (it != dynamic_edges_.end()) return it->second.lookup(stamp_ns);
    return std::nullopt;                                   // edge never set
}

std::optional<Transform> TransformTree::composeUp(FrameId frame, FrameId ancestor,
                                                  int64_t stamp_ns) const {
    Transform acc = Transform::Identity();                 // T_frame_frame
    FrameId f = frame;
    while (f != ancestor) {
        const auto p = frame_parent(f);
        if (!p) return std::nullopt;                       // ancestor not on chain
        const auto e = edgeTransform(f, stamp_ns);         // T_parent_f
        if (!e) return std::nullopt;
        acc = (*e) * acc;                                  // T_parent_frame
        f = *p;
    }
    return acc;                                            // T_ancestor_frame
}

std::optional<FrameId> TransformTree::lowestCommonAncestor(FrameId a, FrameId b) {
    std::vector<FrameId> a_chain;                          // a, parent(a), ..., root
    for (std::optional<FrameId> f = a; f; f = frame_parent(*f))
        a_chain.push_back(*f);
    for (std::optional<FrameId> f = b; f; f = frame_parent(*f)) {
        for (FrameId x : a_chain)
            if (x == *f) return *f;
    }
    return std::nullopt;                                   // disjoint (shouldn't happen)
}

std::optional<Transform> TransformTree::lookupTransform(FrameId target, FrameId source,
                                                        int64_t stamp_ns) const {
    const auto lca = lowestCommonAncestor(target, source);
    if (!lca) return std::nullopt;

    const auto T_lca_source = composeUp(source, *lca, stamp_ns);
    const auto T_lca_target = composeUp(target, *lca, stamp_ns);
    if (!T_lca_source || !T_lca_target) return std::nullopt;

    // T_target_source = T_target_lca * T_lca_source = inv(T_lca_target) * T_lca_source
    return T_lca_target->inverse() * (*T_lca_source);
}

} // namespace kist
