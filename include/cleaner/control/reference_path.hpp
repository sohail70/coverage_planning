#pragma once

#include "cleaner/geometry_types.hpp"
#include <string>
#include <vector>

enum class ReferenceSegmentType {
    COVERAGE,
    TRANSITION
};

struct ReferenceSegment2D {
    Segment<double> segment_;

    // Maximum allowed speed in m/s
    double max_speed_ = 0.0;

    ReferenceSegmentType type_;
};

struct ReferencePath2D {
    // Ordered path in world coordinates (metres)
    std::vector<ReferenceSegment2D> segments_;

    // Coordinate frame, e.g. "map"
    std::string frame_id_;
};
