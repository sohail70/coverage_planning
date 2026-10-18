#pragma once

#include "cleaner/geometry.hpp"
#include "cleaner/weighted_graph.hpp"

class VisibilityGraph {
    public:
        WeightedGraph compute(const Polygon<double>& polygon);

    private:
};