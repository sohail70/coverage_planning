#include "cleaner/transition_planner.hpp"

/*
    Is straight A->B completely inside safe_region?

            YES
            ↓
        use A->B

            NO
            ↓
    visibility graph
    polygon vertices
            ↓
        Dijkstra / A*
            ↓
    shortest valid polyline


    A fixed-altitude drone, omni robot, etc. can use this planner. It simply ignores:
    heading_

*/
class HolonomicPolygonTransitionPlanner : public TransitionPlanner2D {
    public:
        TransitionPath plan(const Pose2D& start, const Pose2D& goal, const Polygon<double>& safe_region) override;
};