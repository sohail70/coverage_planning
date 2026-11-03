#include "cleaner/control/reference_builder.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

ReferenceBuilder::ReferenceBuilder( PointTransform to_world, double coverage_speed, double transition_speed, std::string frame_id)
    : to_world_(std::move(to_world)),
      coverage_speed_(coverage_speed),
      transition_speed_(transition_speed),
      frame_id_(std::move(frame_id)) {

    if (!to_world_)
        throw std::invalid_argument("Missing coordinate transform");

    if (!std::isfinite(coverage_speed_) ||
        !std::isfinite(transition_speed_) ||
        coverage_speed_ <= 0.0 ||
        transition_speed_ <= 0.0)
        throw std::invalid_argument("Invalid speed limits");

    if (frame_id_.empty())
        throw std::invalid_argument("Missing reference frame");
}

ReferencePath2D ReferenceBuilder::build( const GlobalCoveragePlan& plan) const {

    if (plan.traversal_.empty())
        throw std::runtime_error("Global coverage plan is empty");

    ReferencePath2D reference;
    reference.frame_id_ = frame_id_;

    reference.segments_.reserve(plan.traversal_.size());

    constexpr double eps = 1e-8;

    for (const auto& path : plan.traversal_) {

        ReferenceSegment2D ref;

        // Convert planning coordinates to world coordinates
        ref.segment_.start_ = to_world_(path.segment_.start_);
        ref.segment_.end_   = to_world_(path.segment_.end_);

        const auto& a = ref.segment_.start_;
        const auto& b = ref.segment_.end_;

        if (!std::isfinite(a.x_) || !std::isfinite(a.y_) ||
            !std::isfinite(b.x_) || !std::isfinite(b.y_))
            throw std::runtime_error("Non-finite reference point");

        if (std::hypot(b.x_ - a.x_, b.y_ - a.y_) <= eps)
            throw std::runtime_error("Zero-length reference segment");

        switch (path.type_) {
            case PathType::COVERAGE:
                ref.type_ = ReferenceSegmentType::COVERAGE;
                ref.max_speed_ = coverage_speed_;
                break;

            case PathType::TRANSITION:
                ref.type_ = ReferenceSegmentType::TRANSITION;
                ref.max_speed_ = transition_speed_;
                break;

            default:
                throw std::runtime_error("Unknown path type");
        }

        // Ensure consecutive segments connect
        if (!reference.segments_.empty()) {
            const auto& prev =
                reference.segments_.back().segment_.end_;

            if (std::hypot(a.x_ - prev.x_, a.y_ - prev.y_) > eps)
                throw std::runtime_error("Disconnected reference path");
        }

        reference.segments_.push_back(ref);
    }

    return reference;
}
