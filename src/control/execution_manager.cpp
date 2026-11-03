#include "cleaner/control/execution_manager.hpp"
#include "cleaner/control/control_types.hpp"
#include "cleaner/control/path_tracking_controller.hpp"
#include "cleaner/control/reference_path.hpp"

ExecutionManager::ExecutionManager(PathTrackingController& control) : control_(control) {

}

void ExecutionManager::setReferencePath(const ReferencePath2D& ref_path) {
    ref_path_ = ref_path;
}


ExecutionResult ExecutionManager::update(const RobotState& current_state) {
    // determine current path progress
    for (const auto& seg : ref_path_.segments_) {
        std::cout<<"From: "<<seg.segment_.start_<<" to:"<<seg.segment_.end_<<"\n";
    }

}