#pragma once

#include "cleaner/weighted_graph.hpp"
/*
    Traversal does NOT depend on BCD.
    Traversal depends only on WeightedGraph.


    BCD result is an undirected graph with cycle
    DFS/BFS handle that by keeping track of which nodes have already been visited.


    Original WeightedGraph
        may contain cycles
        may have many connections

    DFS starts at one node
        ↓
    produces an implicit rooted DFS tree

    The root is simply whatever you pass:
    start_cell_id

*/
struct TraversalResult
{
    std::vector<cellID> visit_order_;
    std::vector<cellID> walk_;
};
class CellTraversal {
    public:
        virtual ~CellTraversal() = default;

        virtual TraversalResult compute(const WeightedGraph& cells, cellID start_cell_id) = 0;
    private:

};