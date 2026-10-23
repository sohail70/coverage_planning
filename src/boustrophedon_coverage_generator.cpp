#include "cleaner/boustrophedon_coverage_generator.hpp"
#include "cleaner/geometry.hpp"
#include "cleaner/transition_planner.hpp"
#include <algorithm>
#include <iterator>
#include <stdexcept>


BoustrophedonCoverageGenerator::BoustrophedonCoverageGenerator( TransitionPlanner2D& transition_planner) : transition_planner_(transition_planner){

}


std::vector<CellCoverageCandidates> BoustrophedonCoverageGenerator::compute(const std::vector<BCDCell>& bcd_cells, double max_lane_spacing) {

    /**
        coverage-angle normalization should be modulo π, not modulo 2π. In other words, for lane orientation, you only need something like:
                [0, π)
                rather than:
                [-π, π)
                That avoids generating duplicate candidates later.
        so we use zeroToPi instead of piToPi because 
        direction angle:
        0° != 180°
        but in here we are dealing with line orientation and:
        line orientation:
        0° == 180°
    */

    /*
        Ex: 
            Cell	Valid sweep angles	Candidate patterns
            Cell A	0, 45 degree	        Up to 8
            Cell B	30 degree   	        Up to 4
            Cell C	0, 90, 135 degree	    Up to 12
    */
    std::vector<CellCoverageCandidates> region_coverage_candidates_;

    for (int i = 0; i < bcd_cells.size() ; i++) {
        Polygon<double> polygon = bcd_cells.at(i).polygon_;
        std::vector<double> angles_ = polygonSegmentAngles(polygon);
        filterSweepableLaneAnglesForPolygon(angles_, polygon);
        for (auto& angle : angles_) // just a double check! even though i think i arelady do this stuff in polygonSegmentAgnles()
            zeroToPi(angle);
        CellCoverageCandidates  ccc_;
        for (const auto& angle : angles_){
            // For each angle we generate for patters with start-end and direction changes! so one starts from left to right and another from right to left and another two is just the same just starting from the other side! 
            for (int j = 0; j < 4 ; j++){
                CellCoverage cc_ = generateCoverage(bcd_cells.at(i), angle , max_lane_spacing, static_cast<SweepPatternVariant>(j));
                ccc_.cell_id_ = bcd_cells.at(i).id_;
                ccc_.patterns_.push_back(cc_);

            }
        }
        region_coverage_candidates_.push_back(ccc_);
    }

    return region_coverage_candidates_;

}



CellCoverage BoustrophedonCoverageGenerator::generateCoverage(const BCDCell& bcd_cell, double angle, double max_lane_spacing, SweepPatternVariant pattern) {
    if (max_lane_spacing <= 0)
        throw std::runtime_error("generateLanes: lane spacing must be positive");


    Point<double> lane_direction_{cos(angle), sin(angle)};
    // normalize(lane_direction_); // unnecessery because cos^2 + sin^2 = 1
    Point<double> lane_normal_ = normalLeft(lane_direction_);



    double eps_ = 1e-10;
    const auto& polygon_ = bcd_cell.polygon_;
    auto [min_range_ , max_range_] = getProjectionRange(polygon_, lane_normal_);
    double range_ = max_range_ - min_range_;
    if (range_ <= eps_)
        throw std::runtime_error("genrateCoverage: range is zero!");
    int count_ = ceil(range_ / max_lane_spacing);
    double real_lane_spacing_ = range_ / count_;  // Guranteed to be less than the max_allowed_spacing_

    CellCoverage cc_;
    cc_.lane_spacing_ = real_lane_spacing_;
    cc_.angle_ = angle;
    cc_.cell_id_ = bcd_cell.id_;
    cc_.lanes_ = generateLanes( polygon_, lane_normal_, min_range_, count_, real_lane_spacing_);

    buildTraversal(cc_, polygon_, pattern);

    // // buildTraversal establishes the actual ordered robot path,
    // // therefore start_ and end_ become meaningful here.

    // calculateMetrics(cc_);



    return cc_;
}

// angle is the lane angle! in the following function there is no concept of direction of movement! we'll also handle
// the transition between lanes in some other function! 
std::vector<CoverageLane> BoustrophedonCoverageGenerator::generateLanes( const Polygon<double>& polygon, const Point<double>& lane_normal, double min_range, int count, double lane_spacing) {

    std::vector<CoverageLane> coverageLanes_;

    for (int i = 0 ; i < count ; i++) {
        CoverageLane cl_;
        // mind that c is calculated wrt to origin of the coord system (0,0)
        double c = min_range + (i + 0.5)*lane_spacing; // mind that line representation is P.n = c and the info is at the comment section of the linePolygonIntersections function
        cl_.c_=c;
        cl_.segments_ = clipLineToPolygonIntersections(polygon, lane_normal, c);
        coverageLanes_.push_back(cl_);
    }

    return coverageLanes_;

}

/*
    Sort lanes by c_, 
    Within each lane, keep its disconnected segments as separate coverage tasks.
    At the current segment's exit endpoint, evaluate feasible transitions to unvisited segments in the current lane first, then the next lane.
    If those candidates are unsuitable, expand the search to nearby lanes.
    Once all segments have been ordered and oriented, construct traversal_ by inserting the transition paths.
*/

/*
 * Builds one complete boustrophedon traversal for a cell,
 * using its already-generated coverage lanes and the selected
 * sweep pattern variant.
 *
 * Assumptions:
 * - The lanes were generated for a valid sweepable angle.
 * - Each lane contains exactly one connected coverage segment.
 * - The lane offsets c_ describe the lines n . p = c.
 *
 * Implementation steps:
 *
 * 1. Validate the input.
 *    - Handle an empty lane list.
 *    - Verify that each lane has exactly one segment.
 *    - If these assumptions are violated, reject the pattern.
 *
 * 2. Establish a canonical lane order.
 *    - Sort the lanes by ascending c_.
 *    - Prefer to keep cc.lanes_ in this canonical order.
 *    - If the selected variant requires high-to-low progression,
 *      traverse a reversed local sequence of lane references or indices
 *      rather than modifying the stored lane order.
 *
 * 3. Determine the lane traversal direction.
 *    - Calculate the unit vector d from cc.angle_:
 *          d = {cos(angle), sin(angle)}
 *    - The selected variant determines whether the first lane is
 *      traversed along d or against d.
 *    - For each successive lane, alternate the traversal direction.
 *
 * 4. Orient each coverage segment.
 *    - A segment's stored start and end points may not agree with d.
 *    - Use the dot product of (end - start) with d to determine
 *      whether the segment follows or opposes d.
 *    - Select the appropriate entry and exit endpoints for the
 *      current lane traversal.
 *
 * 5. Build the ordered traversal.
 *    - The first lane becomes the first coverage segment.
 *    - For every subsequent lane:
 *        a. Obtain the previous lane's exit point.
 *        b. Obtain the current lane's entry point.
 *        c. Compute a collision-free transition between these points.
 *        d. Append the transition as PathType::TRANSITION.
 *        e. Append the current lane as PathType::COVERAGE.
 *
 * 6. Plan transitions correctly.
 *    - Use a direct connector if the whole segment is valid.
 *    - Otherwise, use the visibility-graph transition planner.
 *    - When matching the paper, transitions should be planned
 *      in the global free space, accounting for obstacles, rather
 *      than being restricted to the current cell polygon.
 *
 * 7. Finalize the pattern.
 *    - Set cc.start_ to the first point of the traversal.
 *    - Set cc.end_ to the final point of the traversal.
 *    - Calculate the complete pattern metrics, including coverage
 *      distance and internal transition distance.
 *
 * Important:
 * - This function constructs one pattern, not all four patterns.
 * - It must preserve every required coverage segment exactly once.
 * - If a required transition cannot be planned, reject this pattern.
 */
// lane-aware greedy method
void BoustrophedonCoverageGenerator::buildTraversal(CellCoverage& cc, const Polygon<double>& polygon, SweepPatternVariant pattern ) {

    cc.traversal_.clear();
    cc.metrics_.feasible_ = false;

    auto lanes = cc.lanes_;
    if (lanes.empty())
        throw std::runtime_error("buildTraversal : lane is empty");


    for (const auto& lane : lanes) {
        if (lane.segments_.size() != 1) {
            throw std::runtime_error(
                "buildTraversal: expected exactly one segment per lane");
        }
    }


    // sort the lanes based on c in P.n = c --> even though this is a formula for line but maybe we can use one point ofthe segment to calculate c
    // I think they are already sorter but doesnt hurt!
    std::sort(lanes.begin(), lanes.end(),
    [](const CoverageLane& A, const CoverageLane& B) {
        return A.c_ < B.c_;
    });


    bool low_to_high_ = false;
    bool first_lane_along_ = false;
    switch (pattern) {
        case SweepPatternVariant::LowToHigh_StartAlongLane :
            low_to_high_ = true;
            first_lane_along_ = true;
            break;
        case SweepPatternVariant::LowToHigh_StartAgainstLane :
            low_to_high_ = true;
            first_lane_along_ = false;
            break;
        case SweepPatternVariant::HighToLow_StartAlongLane : 
            low_to_high_ = false;
            first_lane_along_ = true;
            break;
        case SweepPatternVariant::HighToLow_StartAgainstLane : 
            low_to_high_ = false;
            first_lane_along_ = false;
            break;
        default:
            throw std::runtime_error("buildTraversal : No supported pattern");
            break;
    }

    if (!low_to_high_) {
        std::reverse(lanes.begin(), lanes.end());
    }


    Point<double> lane_dir_{std::cos(cc.angle_), std::sin(cc.angle_)};
    double eps_ = 1e-10;

    auto getLaneEndpoints = [&](const CoverageLane& lane, bool go_along) -> std::pair<Point<double>, Point<double>> {
        const auto& segment = lane.segments_.front();
        const auto& A = segment.start_;
        const auto& B = segment.end_;
        const Point<double> seg_dir = B - A;
        const double len = distance(A, B);
        if (!std::isfinite(len) || len <= eps_) {
            throw std::runtime_error(
                "buildTraversal: invalid coverage segment length");
        }
        const double state = dotProduct2D(seg_dir, lane_dir_);
        if (!std::isfinite(state) || std::abs(state) <= eps_) {
            throw std::runtime_error(
                "buildTraversal: invalid coverage segment direction");
        }
        // Is the segment's A-to-B direction aligned with lane_dir_?
        const bool segment_along = state > 0.0;
        if (go_along == segment_along) {
            return {A, B};
        }
        return {B, A};
    };

    // Handling the first lane
    auto [lane_entry_, lane_exit_] = getLaneEndpoints(lanes.front(), first_lane_along_);


    cc.start_ = lane_entry_;

    PathSegment ps_;
    ps_.type_ = PathType::COVERAGE;
    ps_.segment_.start_ = lane_entry_;
    ps_.segment_.end_ = lane_exit_;

    std::vector<PathSegment> route_;
    route_.push_back(ps_);


    // Now the rest lanes! we start with a transition but we need to again decide the entry point for the next lane!
    // We alternate!
    bool go_along = !first_lane_along_;



    for (size_t i = 1 ; i < lanes.size() ; i++) {
        auto [lane_entry_ , lane_exit_] = getLaneEndpoints(lanes.at(i), go_along);
        Pose2D start_pose_;
        start_pose_.position_ = route_.back().segment_.end_;
        start_pose_.heading_ = 0.0;
        Pose2D end_pose_;
        end_pose_.position_ =lane_entry_;
        end_pose_.heading_ = 0.0;
        auto path = transition_planner_.plan(start_pose_, end_pose_, cc.cell_id_, polygon);
        if (path.waypoints_.size() < 2) {
            throw std::runtime_error(
                "buildTraversal: transition planner found no usable path");
        }
        // sanity check!
        if (distance(path.waypoints_.front(), start_pose_.position_) > eps_ || distance(path.waypoints_.back(), end_pose_.position_) > eps_) {
            throw std::runtime_error( "buildTraversal: transition endpoints do not match");
        }


        Point<double> current = route_.back().segment_.end_;

        for (size_t k = 1; k < path.waypoints_.size(); ++k) { // Start from index 1 to now have duplicates
            if (distance(current, path.waypoints_.at(k)) <= eps_) // Handle zero length
                continue;


            PathSegment transition;
            transition.type_ = PathType::TRANSITION;
            transition.segment_.start_ = current;
            transition.segment_.end_ = path.waypoints_.at(k);

            route_.push_back(transition);
            current = path.waypoints_.at(k);
        }

        ps_.type_ = PathType::COVERAGE;
        ps_.segment_.start_ = lane_entry_;
        ps_.segment_.end_ = lane_exit_;
        route_.push_back(ps_);
        go_along = !go_along;
    }

    cc.traversal_ = std::move(route_);
    cc.end_ = cc.traversal_.back().segment_.end_;

    // cc.metrics_ ?
    calculateMetrics(cc);
    if (!cc.metrics_.feasible_) {
        throw std::runtime_error( "buildTraversal: generated traversal is infeasible");
    }
    
}
/*
    struct CoverageMetrics
    {
        double coverage_distance_	Sum lengths of COVERAGE segments
        double intra_cell_transition_distance_	Sum lengths of TRANSITION segments in this cell's traversal
        double inter_cell_transition_distance_	for the global route optimizer	Sum transitions connecting different cells
        double total_distance_	Sum all traversal segment lengths
        size_t number_of_lanes_	cc.lanes_.size()
        size_t number_of_turns_, but define carefully	Count changes in travel direction or planned turning maneuvers
        double covered_area_	for coverage evaluation	Geometric union of the area swept by the robot
        double uncovered_area_	for coverage evaluation	Cell area minus covered area
        double overlap_area_	Area covered more than once
        bool feasible_	Whether all required coverage and transition constraints are satisfied
    };

*/


void BoustrophedonCoverageGenerator::calculateMetrics(CellCoverage& cc)
{
    double eps_ = 1e-10;
    auto& metrics = cc.metrics_;
    const auto& traversal = cc.traversal_;

    // Reset metrics.
    metrics.coverage_distance_ = 0.0;
    metrics.intra_cell_transition_distance_ = 0.0;
    metrics.inter_cell_transition_distance_ = 0.0;
    metrics.total_distance_ = 0.0;
    metrics.number_of_lanes_ = cc.lanes_.size();
    metrics.feasible_ = false;

    // A valid candidate needs lanes and a traversal.
    if (cc.lanes_.empty() || traversal.empty()) {
        return;
    }

    // Every lane must contain exactly one coverage segment.
    for (const auto& lane : cc.lanes_) {
        if (lane.segments_.size() != 1) {
            return;
        }
    }

    // Check that the traversal contains one coverage segment per lane.
    std::size_t coverage_count = 0;

    for (const auto& path : traversal) {
        if (path.type_ == PathType::COVERAGE) {
            ++coverage_count;
        }
    }

    if (coverage_count != cc.lanes_.size()) {
        return;
    }

    // Validate segments and calculate distances.
    for (const auto& path : traversal) {
        const auto& a = path.segment_.start_;
        const auto& b = path.segment_.end_;

        if (!std::isfinite(a.x_) || !std::isfinite(a.y_) ||
            !std::isfinite(b.x_) || !std::isfinite(b.y_)) {
            return;
        }

        const double segment_length = distance(a, b);

        if (!std::isfinite(segment_length) || segment_length <= eps_) {
            return;
        }

        if (path.type_ == PathType::COVERAGE) {
            metrics.coverage_distance_ += segment_length;
        }
        else if (path.type_ == PathType::TRANSITION) {
            metrics.intra_cell_transition_distance_ += segment_length;
        }
        else {
            // Unexpected path type.
            return;
        }
    }

    // Consecutive segments must connect.
    for (std::size_t i = 1; i < traversal.size(); ++i) {
        const auto& previous = traversal[i - 1].segment_;
        const auto& current = traversal[i].segment_;

        if (distance(previous.end_, current.start_) > eps_) {
            return;
        }
    }

    // Total distance within this cell.
    metrics.total_distance_ =
        metrics.coverage_distance_ +
        metrics.intra_cell_transition_distance_;

    metrics.feasible_ = true;

    // Area metrics and number of turns are handled separately.
    /*
        cc.metrics_.number_of_turns_
        covered_area_: Calculate the union of the regions swept by the robot along the coverage segments.
        uncovered_area_: Subtract the covered region from the free-space cell.
        overlap_area_: Measure the area covered multiple times, using a clearly defined convention.
    */
    /*
        cc.metrics_.feasible_ ? // how to decide?
            There is at least one lane.
            Every lane has exactly one coverage segment.
            Every required coverage segment is included in the traversal.
            Consecutive segments connect without unintended gaps.
            Every transition is valid and remains inside the permitted free-space region.
            The traversal contains no invalid or non-finite coordinates.
    */
}


void BoustrophedonCoverageGenerator::showCoverageCandidates(
    const cv::Mat& original_image,
    const std::vector<CellCoverageCandidates>& candidates) const
{
    if (candidates.empty()) {
        std::cout << "No coverage candidates available.\n";
        return;
    }

    // Prepare a BGR image for drawing.
    cv::Mat display;

    if (original_image.channels() == 1) {
        cv::cvtColor(
            original_image, display, cv::COLOR_GRAY2BGR);
    }
    else if (original_image.channels() == 4) {
        cv::cvtColor(
            original_image, display, cv::COLOR_BGRA2BGR);
    }
    else {
        display = original_image.clone();
    }

    // Convert geometry coordinates to image pixel coordinates.
    // This follows the same -1 offset as showBCDCells().
    auto toPixel = [](const Point<double>& p) {
        return cv::Point(
            cvRound(p.x_ - 1.0),
            cvRound(p.y_ - 1.0));
    };

    // One distinct, deterministic color per cell (BGR).
    auto cellColor = [](int id) {
        return cv::Scalar(
            50 + (id * 53)  % 206,
            50 + (id * 97)  % 206,
            50 + (id * 193) % 206);
    };

    // Random-number generator for candidate selection.
    std::mt19937 rng(std::random_device{}());

    std::size_t plotted_cells = 0;

    for (const auto& cell_candidates : candidates) {

        // Collect all feasible candidates for this cell.
        std::vector<const CellCoverage*> feasible_candidates;
        feasible_candidates.reserve(
            cell_candidates.patterns_.size());

        for (const auto& candidate : cell_candidates.patterns_) {
            if (candidate.metrics_.feasible_) {
                feasible_candidates.push_back(&candidate);
            }
        }

        // Skip cells without a feasible candidate.
        if (feasible_candidates.empty()) {
            std::cout
                << "Cell " << cell_candidates.cell_id_
                << ": no feasible candidate to plot.\n";
            continue;
        }

        // Select one feasible candidate uniformly at random.
        std::uniform_int_distribution<std::size_t> distribution(
            0, feasible_candidates.size() - 1);

        const CellCoverage* selected =
            feasible_candidates[distribution(rng)];

        const cv::Scalar coverage_color =
            cellColor(cell_candidates.cell_id_);

        // Draw the complete traversal for this cell.
        for (const auto& path : selected->traversal_) {

            const cv::Point start =
                toPixel(path.segment_.start_);

            const cv::Point end =
                toPixel(path.segment_.end_);

            if (path.type_ == PathType::COVERAGE) {
                // Coverage lanes: cell-specific color.
                cv::line(
                    display,
                    start,
                    end,
                    coverage_color,
                    2,
                    cv::LINE_AA);
            }
            else if (path.type_ == PathType::TRANSITION) {
                // Intra-cell transitions: yellow.
                cv::line(
                    display,
                    start,
                    end,
                    cv::Scalar(0, 255, 255),
                    2,
                    cv::LINE_AA);
            }
        }

        // Label the cell near its traversal start.
        const cv::Point label_position =
            toPixel(selected->start_);

        const double angle_degrees =
            selected->angle_ * 180.0 / std::acos(-1.0);

        const std::string label =
            "C" + std::to_string(cell_candidates.cell_id_) +
            " " +
            std::to_string(
                static_cast<int>(std::round(angle_degrees))) +
            " deg";

        // Black outline for readable text.
        cv::putText(
            display,
            label,
            label_position,
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(0, 0, 0),
            3,
            cv::LINE_AA);

        cv::putText(
            display,
            label,
            label_position,
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(255, 255, 255),
            1,
            cv::LINE_AA);

        ++plotted_cells;

        std::cout
            << "Plotted cell " << cell_candidates.cell_id_
            << ", angle = " << angle_degrees
            << " deg, lanes = "
            << selected->lanes_.size()
            << ", total intra-cell distance = "
            << selected->metrics_.total_distance_
            << '\n';
    }

    std::cout << "Plotted " << plotted_cells << " cells.\n";

    cv::imshow("Coverage Candidates", display);
    cv::waitKey(0);
}