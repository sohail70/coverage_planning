#pragma once

#include "cleaner/cell_types.hpp"
#include "cleaner/geometry.hpp"

#include <vector>


struct BCDCell
{
    cellID id_;

    // Each polygon is one rectangular slab-piece belonging to this cell.
    /*
        each rectangle (one fragment) is created as:
        0 = top-left
        1 = top-right
        2 = bottom-right
        3 = bottom-left
    */
    std::vector<Polygon<double>> fragments_;


    /*
        So for fragments ordered left-to-right, build:
        top boundary:    left -> right
        bottom boundary: right -> left
    */
    Polygon<double> polygon_; // Unified polygon based on the fragments_;
};