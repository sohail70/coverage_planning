#include "cleaner/global_coverage_planner.hpp"
#include "cleaner/geometry.hpp"

#include <stdexcept>

GlobalCoveragePlan GlobalCoveragePlanner::compute(
    const std::vector<CellCoverageCandidates>& candidates
) const {

    GlobalCoveragePlan plan;
    constexpr double eps = 1e-8;

    if (candidates.empty())
        throw std::runtime_error("No coverage candidates");

    for (const auto& cell : candidates) {

        // Temporary: select the first candidate.
        if (cell.patterns_.empty())
            throw std::runtime_error("Cell has no candidates");

        const CellCoverage& selected = cell.patterns_.front();

        if (selected.cell_id_ != cell.cell_id_ ||
            !selected.metrics_.feasible_ ||
            selected.traversal_.empty()) {
            throw std::runtime_error("Invalid cell candidate");
        }

        // Connect previous cell to current cell.
        if (!plan.traversal_.empty()) {

            const auto& previous_end =
                plan.traversal_.back().segment_.end_;

            const auto& next_start =
                selected.traversal_.front().segment_.start_;

            if (distance(previous_end, next_start) > eps) {

                Segment<double> connection{
                    previous_end, next_start
                };

                if (!is_safe_(connection)) {
                    throw std::runtime_error(
                        "Inter-cell connection is obstructed");
                }

                plan.traversal_.push_back({
                    connection,
                    PathType::TRANSITION
                });
            }
        }

        // Append current cell's complete traversal.
        plan.traversal_.insert(
            plan.traversal_.end(),
            selected.traversal_.begin(),
            selected.traversal_.end()
        );

        plan.cell_order_.push_back(selected.cell_id_);
    }

    return plan;
}
