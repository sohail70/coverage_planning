#pragma once

/*
    Coverage generation = how do I physically cover THIS CELL? 
    Here we work with the results of the BoustrophedonDecomposer's compute function (bcd_cells_) 
    So we have lots of cells and each cell consist of one unified maybe complex polygon_ and we put lanes in there!
*/

#include "cleaner/bcd_type.hpp"
#include "cleaner/geometry.hpp"
#include "cleaner/metrics.hpp"


struct CellCoverageCandidate
{
    cellID cell_id_;

    double angle_;
    double lane_spacing_;

    std::vector<Segment<double>> lanes_;

    Point<double> start_;
    Point<double> end_;

    CoverageMetrics metrics_;
};



class BoustrophedonCoverageGenerator {
    public:
        void compute(const std::vector<BCDCell>& bcd_cells, double angle, double lane_spacing);
        void generateLanes(const Polygon<double>& polygon, double angle, double lane_spacing);
    private:

};
