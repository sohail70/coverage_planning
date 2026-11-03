#pragma once
#include "cleaner/control/control_types.hpp"
#include "cleaner/control/path_tracking_controller.hpp"
#include "cleaner/control/reference_path.hpp"
#include <casadi/casadi.hpp>
#include <casadi/core/dm_fwd.hpp>
#include <casadi/core/function.hpp>
#include <casadi/core/optistack.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


struct MPCConfig {
    // Prediction
    int N = 20;
    double Ts = 0.05;

    // Cost weights
    double q_ct = 100.0;
    double q_heading = 10.0;
    double q_progress = 100.0;
    double r_v = 1.0;
    double r_w = 0.5;
    double w_backward = 50.0;

    // Motion limits
    double max_linear_speed = 0.6;
    double max_angular_speed = 0.7854;
    double a_brake = 0.1;
};


struct MPCParameters {
    casadi::MX x0;

    // Reference segment
    casadi::MX start;
    casadi::MX direction;
    casadi::MX length;
    casadi::MX vmax;
};




class MPC : public PathTrackingController {
    public:
        ControlResult compute(const RobotState& current_state, const ReferencePath2D& local_reference_path) override;
        void reset() override;

        void test1();
        void test2();
        void test3();
        void singleShooting();
        void multipleShooting();
        void multipleShootingTrajectoryTracking();
        void multipleShootingPathTracking();

        casadi::Function dynamics();
        casadi::Function motion();


        void visualizeRobot( const casadi::DM& state, const Segment<double>& reference, const std::vector<cv::Point2d>& robot_path);

    private:
        void buildProblem(); // creates X_, U_ and all symbolic parameters.
        void buildObjective();
        void buildConstraints();
        void updateParameters(const RobotState& state, const ReferencePath2D& reference);
        ControlResult solve();


        MPCConfig config_;
        MPCParameters params_;

        casadi::Opti opti_;
        casadi::MX X_, U_, J_;

        // warm-start guess
        casadi::DM X_guess_, U_guess_;

};