#pragma once
#include "cleaner/control/reference_path.hpp"
#include "cleaner/boustrophedon_coverage_generator.hpp"
#include <stdexcept>

ReferencePath2D makeReferencePath(const CellCoverage& cell_coverage) {
    ReferencePath2D reference_path_;
    reference_path_.segments_.reserve(cell_coverage.traversal_.size());

    for (const auto& path_segment : cell_coverage.traversal_) {
        ReferenceSegment2D ref_seg_;
        ref_seg_.segment_ = path_segment;
        if (path_segment.type_ == PathType::TRANSITION){
            ref_seg_.max_speed_ = 2.0;
        }
        else if (path_segment.type_ == PathType::COVERAGE) {
            ref_seg_.max_speed_ = 5.0;
        }
        else {
            throw std::runtime_error("makeReferencePath : Weird Type");
        }
        reference_path_.segments_.push_back(ref_seg_);
    }
    return reference_path_;
}