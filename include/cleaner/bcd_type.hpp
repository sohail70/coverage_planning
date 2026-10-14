#pragma once

#include "cleaner/geometry.hpp"

#include <vector>

struct BCDCell
{
    int id_;

    // Each polygon is one rectangular slab-piece belonging to this cell.
    std::vector<Polygon<double>> fragments_;



};