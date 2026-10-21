#include "cleaner/holonomic_polygon_transtion_planner.hpp"
#include "cleaner/planning_types.hpp"
#include "cleaner/transition_planner.hpp"
#include <exception>
#include <functional>
#include <iterator>
#include <queue>
#include <unordered_map>

HolonomicPolygonTransitionPlanner::HolonomicPolygonTransitionPlanner(VisibilityGraphCache& graph_cache): graph_cache_(graph_cache) {

}
/*
    1. Check whether the direct segment is valid.
    2. If valid, return the direct path.
    3. Otherwise, obtain the visibility graph.
    4. Connect start and goal to visible graph vertices.
    5. Run shortest-path search.
    6. Return the resulting TransitionPath.
*/
TransitionPath HolonomicPolygonTransitionPlanner::plan(const Pose2D& start,  const Pose2D& goal, cellID id, const Polygon<double>& safe_region) {
    auto state = isSegmentInsidePolygon(Segment<double>{start.position_, goal.position_}, safe_region);
    if (state) {
        TransitionPath tp_;
        tp_.waypoints_.push_back(start.position_);
        tp_.waypoints_.push_back(goal.position_);
        tp_.length_ = distance(start.position_, goal.position_);
        return tp_;
    }
    const WeightedGraph& visibility_graph_ = graph_cache_.getOrBuild(id, safe_region);
    auto query_graph_ = visibility_graph_; // Making a copy to add the start and goal node
    size_t start_node_id_ = query_graph_.nodeCount();
    query_graph_.addNode(start_node_id_);
    size_t goal_node_id_ = start_node_id_ + 1;
    query_graph_.addNode(goal_node_id_);

    /*
        Mind that nodeID is aligned with polygon points_ indicies!
    */
    for (size_t i = 0 ; i < safe_region.points_.size() ; i++) {
        Point<double> p = safe_region.points_.at(i);
        if(isSegmentInsidePolygon(Segment<double>{start.position_, p}, safe_region)) {
            query_graph_.addUndirectedEdge(start_node_id_, i, distance(start.position_,  p));
        }
    }
    /*
        Mind that nodeID is aligned with polygon points_ indicies!
    */
    for (size_t i = 0 ; i < safe_region.points_.size() ; i++) {
        Point<double> p = safe_region.points_.at(i);
        if(isSegmentInsidePolygon(Segment<double>{goal.position_, p}, safe_region)) {
            query_graph_.addUndirectedEdge(goal_node_id_, i, distance(goal.position_,  p));
        }
    }

    auto heuristic = [&](nodeID id){
        Point<double> position_;
        if(id == start_node_id_)
            position_ = start.position_;
        else if (id == goal_node_id_)
            position_ = goal.position_;
        else
            position_ = safe_region.points_.at(id);

        return distance(position_, goal.position_);
    };

    auto [parent, length, found ] = shortestPath(start_node_id_, goal_node_id_, query_graph_, heuristic);

    if(found) {
        
        TransitionPath tp_;
        tp_.length_ = length;
        tp_.waypoints_.push_back(goal.position_);
        auto id_ = parent.at(goal_node_id_);
        while (id_ != start_node_id_) {
            tp_.waypoints_.push_back(safe_region.points_.at(id_));
            id_ = parent.at(id_);
        }
        tp_.waypoints_.push_back(start.position_);
        std::reverse(tp_.waypoints_.begin(), tp_.waypoints_.end());
        return tp_;
        
    }
    return TransitionPath{};


}
std::tuple<std::unordered_map<nodeID, nodeID>, double, bool> HolonomicPolygonTransitionPlanner::shortestPath(nodeID start_node_id, nodeID goal_node_id, const WeightedGraph& graph, const std::function<double(nodeID)>& heuristic) {
    bool found = false;
    // A* shortest path
    struct QueueEntry {
        nodeID id_;
        double g_; // just helps with skipping reptetive nodes with same id and different g_!
        double f_; // g_ + heuristic
    };
    std::unordered_map<nodeID, double> g_;
    std::unordered_map<nodeID, nodeID> parent_;

    struct Compare {
        bool operator() (const QueueEntry& A, const QueueEntry& B) {
            /*
                It means if A has a larger cost than B, A should come after B in priority. 
                Therefore, the smallest f_ ends up at the top.

                compared to sort for basic vector or array etc the ">" is a bit confusing but what the hell we can do!
            */
            return  A.f_ > B.f_;
        }
    };
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, Compare> pq_;
    pq_.push(QueueEntry{start_node_id , 0.0, 0.0 + heuristic(start_node_id)});
    g_.try_emplace(start_node_id, 0);

    while (!pq_.empty()) {
        auto current_ = pq_.top();
        pq_.pop();

        /*
            Suppose node B enters the queue with g_ = 10. Later, you discover a better route to B with g_ = 7.
            - g_[B] now stores 7.
            - The queue still contains the old entry with g_ = 10.
            - When that old entry is popped, 10 > 7 is true, so continue skips it.
            The continue jumps to the next iteration of the while loop.
        */
        auto it = g_.find(current_.id_);
        if (current_.g_ > it->second) // stale! (this node got updated with a better g_ but it was in the queue and we just skip it!)
            continue; 




        if (current_.id_ == goal_node_id) {
            // We reached the goal!
            found = true;
            break;
        }
        for (const auto edge : graph.neighbors(current_.id_)) {
            auto neighbor_id_ = edge.to_;
            auto neighbor_distance_ = edge.weight_;
            auto it = g_.find(neighbor_id_);
            auto new_g_ = g_.at(current_.id_) + neighbor_distance_;
            if (it != g_.end() ) {
                if (new_g_ < it->second){
                    g_[neighbor_id_] = new_g_;
                    parent_[neighbor_id_] = current_.id_; // it can also overright
                    pq_.push(QueueEntry{neighbor_id_, new_g_, new_g_ + heuristic(neighbor_id_)});

                }
                else {
                    // the old g was already better than the new_g_ so do nothing!
                }
            }
            else {
                // First time seeing this neighbor
                g_.emplace(neighbor_id_, new_g_);
                parent_[neighbor_id_] = current_.id_;
                pq_.push(QueueEntry{neighbor_id_, new_g_, new_g_ + heuristic(neighbor_id_)});

            }
            
        }
    }

    if (!found)
        return {parent_, 0.0, false};

    return {parent_, g_.at(goal_node_id), true};

}