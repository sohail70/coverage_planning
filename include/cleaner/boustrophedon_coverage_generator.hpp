#pragma once

/*
    Coverage generation = how do I physically cover THIS CELL? 
    Here we work with the results of the BoustrophedonDecomposer's compute function (bcd_cells_) 
    So we have lots of cells and each cell consist of one unified maybe complex polygon_ and we put lanes in there!
*/

#include "cleaner/bcd_type.hpp"
#include "cleaner/geometry.hpp"
#include "cleaner/metrics.hpp"


struct CoverageLane
{
    double c_; // Signed normal offset of this sweep line from the geometry coordinate origin (0,0), assuming the normal is unit length.
    std::vector<Segment<double>> segments_;
};

enum class PathType
{
    COVERAGE,
    TRANSITION
};

struct PathSegment
{
    Segment<double> segment_;
    PathType type_;
};

struct CellCoverage
{
    cellID cell_id_;

    double angle_;
    double lane_spacing_; // actual spacing used, not necessarily requested max

    /*
        cc_.lanes_
        = geometry that MUST be covered
        = do not reorder/mutate it to represent robot motion
    */

    std::vector<CoverageLane> lanes_;
    /*
        cc_.traversal_
            = actual ordered robot path
            = coverage + transitions
    */
    std::vector<PathSegment> traversal_;


    Point<double> start_;
    Point<double> end_;

    CoverageMetrics metrics_;
};



class BoustrophedonCoverageGenerator {
    public:
        std::vector<CellCoverage> compute( const std::vector<BCDCell>& bcd_cells, const std::vector<double>& angles, double max_lane_spacing);
        CellCoverage generateCoverage(const BCDCell& bcd_cells, double angle, double max_lane_spacing);
        std::vector<CoverageLane> generateLanes( const Polygon<double>& polygon, const Point<double>& lane_normal, double min_range, int count, double lane_spacing);
    private:
        void findTransition( Point<double> from, Point<double> to, Polygon<double> cell_polygon);


};


