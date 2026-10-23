#pragma once

/*
    Coverage generation = how do I physically cover THIS CELL? 
    Here we work with the results of the BoustrophedonDecomposer's compute function (bcd_cells_) 
    So we have lots of cells and each cell consist of one unified maybe complex polygon_ and we put lanes in there!
*/

#include "cleaner/bcd_type.hpp"
#include "cleaner/geometry.hpp"
#include "cleaner/metrics.hpp"
#include "cleaner/transition_planner.hpp"
#include "opencv2/opencv.hpp"
#include <random>
struct CoverageLane
{
    double c_; // Signed normal offset of this sweep line from the geometry coordinate origin (0,0), assuming the normal is unit length.
    std::vector<Segment<double>> segments_;
};

/*
    LowToHigh: visit lanes in increasing c_ order.
    HighToLow: visit lanes in decreasing c_ order.
    StartAlongLane: traverse the first lane along lane_direction.
    StartAgainstLane: traverse the first lane against lane_direction.
*/
enum class SweepPatternVariant {
    LowToHigh_StartAlongLane,
    LowToHigh_StartAgainstLane,
    HighToLow_StartAlongLane,
    HighToLow_StartAgainstLane
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

struct CellCoverageCandidates {
    cellID cell_id_;
    std::vector<CellCoverage> patterns_;
};


class BoustrophedonCoverageGenerator {
    public:
        explicit BoustrophedonCoverageGenerator( TransitionPlanner2D& transition_planner);

        std::vector<CellCoverageCandidates> compute( const std::vector<BCDCell>& bcd_cells, double max_lane_spacing);
        CellCoverage generateCoverage(const BCDCell& bcd_cells, double angle, double max_lane_spacing, SweepPatternVariant pattern);
        std::vector<CoverageLane> generateLanes( const Polygon<double>& polygon, const Point<double>& lane_normal, double min_range, int count, double lane_spacing);

        void buildTraversal(CellCoverage& cc, const Polygon<double>& polygon, SweepPatternVariant pattern);
        void calculateMetrics(CellCoverage& cc);
        void showCoverageCandidates( const cv::Mat& original_image, const std::vector<CellCoverageCandidates>& candidates) const;

    private:
        TransitionPlanner2D& transition_planner_;


};


