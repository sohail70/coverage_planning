#pragma once
#include "cleaner/control/path_tracking_controller.hpp"
#include <casadi/casadi.hpp>
#include <casadi/core/function.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


/**
 * APPROACH 1: TRAJECTORY TRACKING (TIME-DEPENDENT)
 *
 * Tracks a time-parameterized reference trajectory.
 * Each reference state specifies where the robot should be
 * at a particular time step.
 *
 * At each MPC iteration, the reference horizon advances with time.
 *
 * Objective:
 *
 *   J = sum(k=0 to N-1) [
 *           (X_k - X_ref,k)^T * Q * (X_k - X_ref,k)
 *         + (U_k - U_ref,k)^T * R * (U_k - U_ref,k)
 *       ]
 *       + (X_N - X_ref,N)^T * Q_f * (X_N - X_ref,N)
 *
 * X_k     : Predicted robot state at time step k.
 * X_ref,k : Reference robot state at time step k.
 * U_k     : Predicted control input.
 * U_ref,k : Reference control input (optional).
 * Q, R    : State and control weighting matrices.
 * Q_f     : Terminal state weighting matrix.
 * N       : Prediction horizon.
 *
 * The robot is penalized for deviating from its desired
 * position and motion at each time step.
 *
 */


/**
 * APPROACH 2: GEOMETRIC PATH FOLLOWING (TIME-INDEPENDENT)
 *
 * Tracks a geometric path represented by a polyline
 * (a sequence of waypoints connected by line segments).
 *
 * At each MPC iteration, calculate the minimum distance
 * between the predicted robot position and the path.
 *
 * Objective:
 *
 *   J = sum(k=0 to N-1) [
 *           w_e * d(P_k, C)^2
 *         + U_k^T * R * U_k
 *       ]
 *       + w_f * d(P_N, C)^2
 *
 * d(P_k, C) : Minimum Euclidean distance from predicted
 *             robot position to the polyline.
 * w_e       : Path tracking weight.
 * w_f       : Terminal tracking weight.
 * U_k       : Control input.
 * R         : Control effort weighting matrix.
 * C         : Geometric reference path.
 *
 * There is no predefined arrival time. The robot is free
 * to move along the path at its own pace.
 *
 */


/**
 * APPROACH 3: PATH FOLLOWING WITH PROGRESS (TIME-INDEPENDENT)
 *
 * Tracks a geometric path while explicitly encouraging
 * the robot to make forward progress along it.
 *
 * Represent the path using arc length s, where s = 0
 * corresponds to the beginning of the path.
 *
 * Maintain an estimated current progress s_current.
 * Use a local path neighborhood to avoid jumping to
 * unrelated or nearby path segments.
 *
 * Objective:
 *
 *   J = sum(k=0 to N-1) [
 *           w_e * e_perp,k^2
 *         + w_s * (s_k - s_target,k)^2
 *         + U_k^T * R * U_k
 *       ]
 *       + w_f * e_perp,N^2
 *       + w_sf * (s_N - s_target,N)^2
 *
 * e_perp,k  : Cross-track error (perpendicular deviation
 *             from the reference path).
 * s_k       : Predicted path progress at step k.
 * s_target,k: Desired path progress at step k.
 * w_e       : Cross-track error weight.
 * w_s       : Progress tracking weight.
 * w_f, w_sf : Terminal weights.
 *
 * The progress objective encourages the robot to advance
 * along the path instead of simply staying close to it.
 *
 */

class MPC : public PathTrackingController {
    public:
        ControlCommand compute(const RobotState& current_state, const ReferencePath2D& reference_path) override;
        void reset() override;

        void test1();
        void test2();
        void test3();
        void singleShooting();
        void multipleShooting();
        void multipleShootingTrajectoryTracking();
        void multipleShootingPathTracking();

        casadi::Function dynamics();


        void visualizeRobot( const casadi::DM& state, const Segment<double>& reference, const std::vector<cv::Point2d>& robot_path);

    private:
};