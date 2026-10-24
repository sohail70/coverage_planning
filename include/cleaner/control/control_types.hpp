#pragma once
#include <vector>

struct RobotState {
    // State variables defined by the selected robot model.
    std::vector<double> values_;
};

struct ControlCommand {
    // Control inputs defined by the selected robot model.
    std::vector<double> values_;
};