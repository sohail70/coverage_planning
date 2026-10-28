#pragma once
#include "cleaner/control/path_tracking_controller.hpp"
#include <casadi/casadi.hpp>
#include <casadi/core/function.hpp>

class MPC : public PathTrackingController {
    public:
        ControlCommand compute(const RobotState& current_state, const ReferencePath2D& reference_path) override;
        void reset() override;

        void test1();
        void test2();
        void test3();
        void singleShooting();
        void multipleShooting();
        void multipleShootingTracking();

        casadi::Function dynamics();

    private:
};