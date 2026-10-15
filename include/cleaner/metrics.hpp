#pragma once

#include <cstddef>

struct CoverageMetrics
{
    // -------------------------------------------------
    // Distance
    // -------------------------------------------------

    double coverage_distance_ = 0.0;

    // Connections between consecutive lanes
    // inside the same cell.
    double intra_cell_transition_distance_ = 0.0;

    // Travel between different cells.
    // For a single-cell candidate this will normally be 0.
    double inter_cell_transition_distance_ = 0.0;

    double total_distance_ = 0.0;


    // Motion complexity

    std::size_t number_of_lanes_ = 0;

    std::size_t number_of_turns_ = 0;


    // Coverage quality

    double covered_area_ = 0.0;

    double uncovered_area_ = 0.0;

    double overlap_area_ = 0.0;


    // Feasibility

    bool feasible_ = true;
};


struct CoverageParameters
{
    double angle_ = 0.0;
    double lane_spacing_ = 0.0;
};