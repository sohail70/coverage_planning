#pragma once
#include "cleaner/geometry_types.hpp"



struct ReferenceSegment2D {
    Segment<double> segment_;
    // Maximum allowed speed along this segment, in m/s.
    double max_speed_ = 0.0;
};

struct ReferencePath2D {
    // Ordered segments the controller must follow.
    std::vector<ReferenceSegment2D> segments_;
};