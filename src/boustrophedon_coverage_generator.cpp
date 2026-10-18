#include "cleaner/boustrophedon_coverage_generator.hpp"
#include "cleaner/geometry.hpp"
#include <stdexcept>




std::vector<CellCoverage> BoustrophedonCoverageGenerator::compute(const std::vector<BCDCell>& bcd_cells, const std::vector<double>& cell_angles, double max_lane_spacing) {

    auto normalized_cell_angles_ = cell_angles;
    /**
        coverage-angle normalization should be modulo π, not modulo 2π. In other words, for lane orientation, you only need something like:
                [0, π)
                rather than:
                [-π, π)
                That avoids generating duplicate candidates later.
        so we use zeroToPi instead of piToPi because 
        direction angle:
        0° != 180°
        but in here we are dealing with line orientation and:
        line orientation:
        0° == 180°
    */
    for (auto& angle : normalized_cell_angles_)
        zeroToPi(angle);

    std::vector<CellCoverage> cell_coverages_;

    for (int i = 0; i < bcd_cells.size() ; i++) {
        CellCoverage cc = generateCoverage(bcd_cells.at(i), normalized_cell_angles_.at(i), max_lane_spacing);
        cell_coverages_.push_back(cc);
    }

    return cell_coverages_;

}



CellCoverage BoustrophedonCoverageGenerator::generateCoverage(const BCDCell& bcd_cell, double angle, double max_lane_spacing) {
    if (max_lane_spacing <= 0)
        throw std::runtime_error("generateLanes: lane spacing must be positive");

    Point<double> lane_direction_{cos(angle), sin(angle)};
    // normalize(lane_direction_); // unnecessery because cos^2 + sin^2 = 1
    Point<double> lane_normal_ = normalLeft(lane_direction_);



    double eps_ = 1e-10;
    CellCoverage cc_;
    const auto& polygon_ = bcd_cell.polygon_;
    auto [min_range_ , max_range_] = getProjectionRange(polygon_, lane_normal_);
    double range_ = max_range_ - min_range_;
    if (range_ == eps_)
        throw std::runtime_error("genrateCoverage: range is zero!");
    int count_ = ceil(range_ / max_lane_spacing);
    double real_lane_spacing_ = range_ / count_;  // Guranteed to be less than the max_allowed_spacing_
    cc_.lane_spacing_ = real_lane_spacing_;
    cc_.angle_ = angle;
    cc_.cell_id_ = bcd_cell.id_;
    cc_.lanes_ = generateLanes( polygon_, lane_normal_, min_range_, count_, real_lane_spacing_);


    // buildTraversal(cc_, polygon_);

    // // buildTraversal establishes the actual ordered robot path,
    // // therefore start_ and end_ become meaningful here.

    // calculateMetrics(cc_);



    return cc_;
}

// angle is the lane angle! in the following function there is no concept of direction of movement! we'll also handle
// the transition between lanes in some other function! 
std::vector<CoverageLane> BoustrophedonCoverageGenerator::generateLanes( const Polygon<double>& polygon, const Point<double>& lane_normal, double min_range, int count, double lane_spacing) {

    std::vector<CoverageLane> coverageLanes_;

    for (int i = 0 ; i < count ; i++) {
        CoverageLane cl_;
        // mind that c is calculated wrt to origin of the coord system (0,0)
        double c = min_range + (i + 0.5)*lane_spacing; // mind that line representation is P.n = c and the info is at the comment section of the linePolygonIntersections function
        cl_.c_=c;
        cl_.segments_ = clipLineToPolygonIntersections(polygon, lane_normal, c);
        coverageLanes_.push_back(cl_);
    }

    return coverageLanes_;

}



void BoustrophedonCoverageGenerator::findTransition( Point<double> from, Point<double> to, Polygon<double> cell_polygon) {

}