#pragma once

#include "cleaner/geometry.hpp"
#include "cleaner/weighted_graph.hpp"

class VisibilityGraph {
    public:
        WeightedGraph compute(const Polygon<double>& polygon);
        bool isVisible(const Point<double>& A , const Point<double>& B, const Polygon<double>& polygon);
    private:

};


class VisibilityGraphCache {
    public:
        const WeightedGraph& getOrBuild( cellID id, const Polygon<double>& polygon);

    private:
        std::unordered_map<cellID, WeightedGraph> graphs_;
};