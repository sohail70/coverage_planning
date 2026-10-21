#include "cleaner/transition_planner.hpp"
#include "cleaner/visibility_graph.hpp"
#include "cleaner/weighted_graph.hpp"
#include <functional>
#include <queue>
#include <tuple>
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
        explicit HolonomicPolygonTransitionPlanner(VisibilityGraphCache& graph_cache);
        TransitionPath plan(const Pose2D& start, const Pose2D& goal, cellID id, const Polygon<double>& safe_region) override;
        std::tuple<std::unordered_map<nodeID, nodeID>, double, bool> shortestPath(nodeID start_node_id, nodeID goal_node_id, const WeightedGraph& vg, const std::function<double(nodeID)>& heuristic);
    
    private:
        VisibilityGraphCache& graph_cache_;
};