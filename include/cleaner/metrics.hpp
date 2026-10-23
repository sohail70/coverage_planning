#pragma once

#include <cstddef>

/*
    Metric	How to calculate
    coverage_distance_	Sum lengths of COVERAGE segments
    intra_cell_transition_distance_	Sum lengths of TRANSITION segments in this cell's traversal
    inter_cell_transition_distance_	for the global route optimizer	Sum transitions connecting different cells
    total_distance_	Sum all traversal segment lengths
    number_of_lanes_	cc.lanes_.size()
    number_of_turns_, but define carefully	Count changes in travel direction or planned turning maneuvers
    covered_area_	for coverage evaluation	Geometric union of the area swept by the robot
    uncovered_area_	for coverage evaluation	Cell area minus covered area
    overlap_area_	Area covered more than once
    feasible_	Whether all required coverage and transition constraints are satisfied
*/
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