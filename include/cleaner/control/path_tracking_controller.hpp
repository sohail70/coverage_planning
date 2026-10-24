#pragma once
#include "cleaner/control/control_types.hpp"
#include "cleaner/control/reference_path.hpp"

#include <experimental/filesystem>
class PathTrackingController {
    public:
        virtual ~PathTrackingController() = default;
        virtual ControlCommand compute(const RobotState& current_state, const ReferencePath2D& reference_path) = 0;
        virtual void reset() = 0; // because controllers may retain internal state, such as MPC's previous solution for warm starting or a controller's progress along the path.

    private:

};