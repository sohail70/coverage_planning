#pragma once
/*
    Given a start robot state, a goal robot state, and a safe region, 
    find a feasible transition path and its cost.
*/
#include "cleaner/geometry.hpp"

struct Pose2D {
    Point<double> position_;
    double heading_;
};

struct TransitionPath {
    bool feasible_ = false;
    std::vector<Point<double>> waypoints_;
    double length_ = 0.0;
};


class TransitionPlanner2D {
    public:
        virtual ~TransitionPlanner2D() = default;

        virtual TransitionPath plan(const Pose2D& start, const Pose2D& goal, const Polygon<double>& safe_region) = 0;

    private:
};