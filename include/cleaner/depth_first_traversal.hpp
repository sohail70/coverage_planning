#pragma once
#include "cleaner/cell_graph.hpp"
#include "cleaner/cell_traversal.hpp"
#include <unordered_set>
#include <vector>

class DepthFirstTraversal : public CellTraversal {
    public:
        TraversalResult compute(const CellGraph& cells, cellID start_cell_id) override;
        void dfs(const CellGraph& cells, cellID current_cell_id, std::unordered_set<cellID>& visited, TraversalResult& res_);
    private:
};