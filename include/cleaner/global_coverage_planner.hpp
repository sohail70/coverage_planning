#pragma once

#include "cleaner/boustrophedon_coverage_generator.hpp"
#include <functional>
#include <utility>

/*
    This happens after BoustrophedonCoverageGenerator and figures out the best 
    choice between the combination of candidates!
*/





struct GlobalCoveragePlan {
    std::vector<PathSegment> traversal_;
    std::vector<cellID> cell_order_;
};

class GlobalCoveragePlanner {
public:
    using CollisionChecker =
        std::function<bool(const Segment<double>&)>;

    explicit GlobalCoveragePlanner(CollisionChecker checker)
        : is_safe_(std::move(checker)) {}

    GlobalCoveragePlan compute(
        const std::vector<CellCoverageCandidates>& candidates
    ) const;

private:
    CollisionChecker is_safe_;
};
