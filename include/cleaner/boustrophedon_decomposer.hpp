#pragma once
#include <opencv2/core/types.hpp>
#include <vector>
#include "cleaner/map_geometry.hpp"
#include "cleaner/bcd_type.hpp"
#include "cleaner/cell_graph.hpp"

class BoustrophedonDecomposer {
    public:

        void compute(const PolygonWithHoles<int>& polygon); // in the end it fills the bcd_cells_

        const CellGraph& getGraph() const;
        const std::vector<BCDCell>& getCells() const;

        // Debug Plots
        void showVerticalSlices( const cv::Mat& original_image);
        void showBCDCellIDs(const cv::Mat& original_image);
        cv::Scalar colorFromCellId(int cell_id) const;
        void showBCDCells(const cv::Mat& original_image);
    private:

        // Why private? future CoveragePlanner should never care that BCD was calculated using midpoint vertical slices. It should only receive BCDCell
        struct VerticalSlice {
            double x_;        // midpoint, useful for intersection sampling

            double x_left_;   // critical_x[i]
            double x_right_;  // critical_x[i+1]


            std::vector<Segment<double>> free_intervals_;

            /*
                Invariant:
                free_intervals_[i] <-> cell_ids_[i]
                A continuing interval inherits its previous cell ID.
                New IDs are created at starts, splits and merges.
            */

            // Same index as free_intervals_ --> every interval gets a brand-new cell ID.
            // Tells which BCD cell each interval currently belongs to.
            // The invariant is simply: free_intervals_[i]  <->  cell_ids_[i]
            std::vector<int> cell_ids_;
        };


        struct IntervalConnection {
            int left_interval_; // Index on one verticalSlice.free_intervals_;
            int right_interval_;
        };


        std::vector<BoustrophedonDecomposer::VerticalSlice> vertical_slices_;

        std::vector<BCDCell> bcd_cells_;

        CellGraph graph_;

        std::vector<BoustrophedonDecomposer::IntervalConnection> overlapGraph(const BoustrophedonDecomposer::VerticalSlice& slice_left , const BoustrophedonDecomposer::VerticalSlice& slice_right);
        void propagateCellIDs(const std::vector<std::vector<int>>& successor , const std::vector<std::vector<int>>& predecessors, BoustrophedonDecomposer::VerticalSlice& slice_left, BoustrophedonDecomposer::VerticalSlice& slice_right, int& next_cell_id_);
        std::vector<Point<double>> getYIntersections(double x, const PolygonWithHoles<int>& polygon); //vertical sweeping line intersections with a polygon

};