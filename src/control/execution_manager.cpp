#include "cleaner/control/execution_manager.hpp"
#include "cleaner/control/control_types.hpp"
#include "cleaner/control/path_tracking_controller.hpp"
#include "cleaner/control/reference_path.hpp"
#include "cleaner/geometry.hpp"

ExecutionManager::ExecutionManager(PathTrackingController& control) : control_(control) {

}

void ExecutionManager::setReferencePath(const ReferencePath2D& ref_path) {
    ref_path_ = ref_path;
    current_segment_index_ = 0;
    status_ = ExecutionStatus::RUNNING;
    control_.reset();
}



ExecutionResult ExecutionManager::update(const RobotState& current_state) {
    // determine current path progress

    ExecutionResult er_;
    er_.status_ = ExecutionStatus::IDLE;

    // No reference path
    if (ref_path_.segments_.empty()) {
        return er_;
    }

    // Check if all segments are completed
    if (current_segment_index_ >= ref_path_.segments_.size()) {
        er_.status_ = ExecutionStatus::COMPLETED;
        return er_;
    }

    // Determine current path progress
    const auto& current_ref_ = ref_path_.segments_.at(current_segment_index_);
    const auto& current_seg_ = current_ref_.segment_;

    auto dir = current_seg_.end_ - current_seg_.start_;
    normalize(dir);

    // Project the current robot position onto the current segment
    double distance_along_seg_ =
        dotProduct2D(current_state.pose_.position_ - current_seg_.start_, dir);

    double remaining = distance(current_seg_.start_, current_seg_.end_) - distance_along_seg_;


    std::cout << "Active segment: " << current_segment_index_ << "\n";
    std::cout << "Remaining: " << remaining << "\n";
    std::cout << "Robot position: " << current_state.pose_.position_ << "\n";
    std::cout << "Robot heading: " << current_state.pose_.heading_ << "\n";


    // Advance only when the robot has reached the segment endpoint.
    // Measured along the segment only: a robot slightly off the line could otherwise never get
    // close enough to the endpoint, and a large tolerance makes it turn before the corner.
    // See the comment above MPC::buildObjective().
    //
    // OLD (wrong), with position_tolerance_ = 0.15:
    //     The robot started turning 0.15 m before the corner, so it joined the next line about 0.13 m to the side.
    //     Lowering the tolerance alone made it stall, because endpoint_distance includes the sideways error:
    //     a robot 0.04 m off the line can never get within 0.03 m of the endpoint.
    //
    //     double endpoint_distance = distance(current_state.pose_.position_, current_seg_.end_);
    //     if (remaining <= position_tolerance_ && endpoint_distance <= position_tolerance_) {
    if (remaining <= position_tolerance_) {

        current_segment_index_++;
        if (current_segment_index_ >= ref_path_.segments_.size()) {
            er_.status_ = ExecutionStatus::COMPLETED;
            return er_;
        }
    }

    // Build the local reference path (one segment for now) --> Driving through corners with stopping 
    // If driving through corners without stopping is on your mind we should use lookahead and send multi segment so the robot has some future context
    ReferencePath2D local_reference_path;
    local_reference_path.frame_id_ = ref_path_.frame_id_;
    local_reference_path.segments_.push_back(
        ref_path_.segments_.at(current_segment_index_)
    );

    // Compute control command
    auto result = control_.compute(current_state, local_reference_path);

    er_.command_ = result.command_;
    er_.status_ = result.status_ == ControlStatus::SUCCESS
        ? ExecutionStatus::RUNNING
        : ExecutionStatus::FAILED;

    return er_;
}
