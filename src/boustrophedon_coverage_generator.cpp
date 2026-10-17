#include "cleaner/boustrophedon_coverage_generator.hpp"
#include "cleaner/geometry.hpp"


void BoustrophedonCoverageGenerator::compute(const std::vector<BCDCell>& bcd_cells, double angle, double lane_spacing) {
    for (const auto& cell : bcd_cells){
        auto polygon_ = cell.polygon_;
        generateLanes(polygon_, angle, lane_spacing);
    }
}

void BoustrophedonCoverageGenerator::generateLanes(const Polygon<double>& polygon, double angle, double lane_spacing) {
    piToPi(angle);
    Point<double> lane_direction_{cos(angle), sin(angle)};
    normalize(lane_direction_);
    Point<double> lane_normal_ = normalLeft(lane_direction_);

    auto [min , max] = getProjectionRange(polygon, lane_normal_);
}