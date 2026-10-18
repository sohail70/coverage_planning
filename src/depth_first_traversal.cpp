#include "cleaner/depth_first_traversal.hpp"
#include "cleaner/weighted_graph.hpp"
#include "cleaner/cell_traversal.hpp"
#include <unordered_set>


TraversalResult DepthFirstTraversal::compute(const WeightedGraph& cells, cellID start_cell_id) {
    TraversalResult res_;

    // Have I already discovered this node?
    std::unordered_set<cellID> visited;

    res_.visit_order_.push_back(start_cell_id);
    res_.walk_.push_back(start_cell_id);

    visited.insert(start_cell_id);

    int current_cell_id_ = start_cell_id;

    // dfs(0)
    dfs(cells, current_cell_id_, visited, res_);
    

    // LOG
    std::cout << "\n========== DFS RESULT ==========\n";
    std::cout << "Visit order: ";
    for (const auto& id : res_.visit_order_) {
        std::cout << id << " ";
    }
    std::cout << "\nWalk: ";
    for (const auto& id : res_.walk_) {
        std::cout << id << " ";
    }
    std::cout << "\n";
    std::cout << "Visited cells: " << res_.visit_order_.size() << "\n";
    std::cout << "Walk length in graph steps: ";
    if (!res_.walk_.empty())
        std::cout << res_.walk_.size() - 1;
    else
        std::cout << 0;
    std::cout << "\n================================\n";




    return res_;

}


/*
    0
    |
    1
   / \
  2   3

  
  dfs(0)
    dfs(1)
        dfs(2)
        return to 1

        dfs(3)
        return to 1
    return to 0


    visit_order_: 0, 1, 2, 3
    walk_: 0, 1, 2, 1, 3, 1, 0
*/
void DepthFirstTraversal::dfs(const WeightedGraph& cells, cellID current_cell_id, std::unordered_set<cellID>& visited, TraversalResult& res_) {
    const auto& neighbors_ = cells.neighbors(current_cell_id);
    for (const auto& n : neighbors_ ) {
        auto iter = std::find(visited.begin(), visited.end(), n.to_);
        if (iter == visited.end()){
            res_.visit_order_.push_back(n.to_);
            visited.insert(n.to_);

            /*
                Encounter visited neighbor:
                    "I see an edge to a node I've already explored."
                    NO MOVEMENT

                Return from recursive child:
                    "I entered this child, finished its branch,
                    and now must return to the parent."
                    BACKTRACK MOVEMENT
            */
            // We are at 1
            // Move 1 -> 2
            res_.walk_.push_back(n.to_);
            dfs(cells, n.to_, visited, res_);
            // dfs(2) is finished.
            // We are returning 2 -> 1
            res_.walk_.push_back(current_cell_id); // backtrack
        }
        else{
            // std::cout<<"Already visited" <<*iter<<"\n";
        }
    }

}