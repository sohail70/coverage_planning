#pragma once

#include "cleaner/control/reference_path.hpp"
#include "cleaner/global_coverage_planner.hpp"
#include <functional>
#include <string>

/*
    Job: convert GlobalCoveragePlan → ReferencePath2D
*/

class ReferenceBuilder {
public:
    using PointTransform = std::function<Point<double>(const Point<double>&)>;

    // dependency injection for coordinate conversion. The builder doesn't need to know whether the map originated from OpenCV or another source.
    ReferenceBuilder(PointTransform to_world, double coverage_speed, double transition_speed, std::string frame_id = "map");

    ReferencePath2D build( const GlobalCoveragePlan& plan) const;

private:
    PointTransform to_world_;

    double coverage_speed_;
    double transition_speed_;

    std::string frame_id_;
};
