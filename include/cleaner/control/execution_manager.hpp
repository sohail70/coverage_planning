#pragma once

#include "cleaner/control/control_types.hpp"
#include "cleaner/control/path_tracking_controller.hpp"
#include "cleaner/control/reference_path.hpp"
/*
    You can inject an MPC implementation when constructing the manager, 
    and later substitute another controller without changing its execution logic.


    Receive a global path from coverage_core.
    Track progress — determine which portion of the path the robot is currently executing.
    Provide a local reference — select the next few segments for the controller.
    Handle path updates — replace or update the active path when replanning occurs.
    Manage controller lifecycle — initialize/reset the controller when necessary and pass the current state and reference to it.
    Report execution status — running, completed, blocked, failed, etc.
*/


class ExecutionManager {
    public:
        ExecutionManager(PathTrackingController& control);
        void setReferencePath(const ReferencePath2D& ref_path);
        ExecutionResult update(const RobotState& current_state);
    private:
        PathTrackingController& control_;
        ReferencePath2D ref_path_;
        int current_segment_index_;
        ExecutionStatus status_;
        double lookahead_distance_;

};