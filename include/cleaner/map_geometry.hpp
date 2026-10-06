#pragma once
#include "cleaner/occupancy_grid.hpp"
#include <array>
#include <vector>
#include <algorithm>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <optional>

/*
 * Coordinate conventions used in MapGeometry
 * ------------------------------------------
 *
 * working_grid_ is indexed like an image:
 *
 *      working_grid_[row][col]
 *      working_grid_[y][x]
 *
 * So:
 *      row = y
 *      col = x
 *
 * Point<int> stores Cartesian-style coordinates:
 *
 *      Point<int>{x, y}
 *      Point<int>{col, row}
 *
 * Therefore:
 *
 *      grid access:
 *          working_grid_[p.y_][p.x_]
 *
 *      create Point from row/col:
 *          Point<int>{col, row}
 *
 *      create row/col from Point:
 *          row = p.y_
 *          col = p.x_
 *
 * Neighbor movement:
 *
 *      Point<int> step{dx, dy}
 *
 *      next.x_ = current.x_ + step.x_
 *      next.y_ = current.y_ + step.y_
 *
 * IMPORTANT:
 *      Never use Point{row, col}.
 *      Point is always {x, y} = {col, row}.
 */



/**
 * Extracts traversable free-space geometry from an OccupancyGrid.
 *
 * The input OccupancyGrid may come from:
 *   - rasterized image
 *   - ROS occupancy grid
 *   - SLAM
 *
 * MapGeometry does not know or care about the source.
 *
 * Output:
 *   - outer free-space boundaries
 *   - obstacle holes inside free space


 *   XXXXXXXXXXXXXXXXXXXX
 *   X                  X
 *   X      XXXX        X
 *   X      XXXX        X
 *   X                  X
 *   XXXXXXXXXXXXXXXXXXXX
 *   Contour 1
 *       is_hole = false
 *       outer boundary of free region
 *
 *   Contour 2
 *       is_hole = true
 *       boundary around XXXX
 *       parent = Contour 1

 */


 /*
    left_boundary   -> left EDGE of current_ belongs to this contour
    top_boundary    -> top EDGE of current_ belongs to this contour
    right_boundary  -> right EDGE of current_ belongs to this contour
    bottom_boundary -> bottom EDGE of current_ belongs to this contour
 */
struct BoundarySides {
    bool left_   = false;
    bool top_    = false;
    bool right_  = false;
    bool bottom_ = false;
};


 // Raster Level Data
struct Contour{
    int id_;
    std::vector<Point<int>> points_;

    // Same index as points_.
    // Describes which cell edges belong to this contour occurrence.
    std::vector<BoundarySides> boundary_sides_;

    int parent_id_;
    /*
        is_hole_ == false
            boundary of a free-space component

        is_hole_ == true
            occupied/unknown hole inside that free-space component
    */
    bool is_hole_;

};

// Geometry Level Data
class Polygon {
public:
    std::vector<Point<double>> points_;
};

struct PolygonWithHoles {
    Polygon outer_;
    std::vector<Polygon> holes_;
};

class MapGeometry{
    public:
        MapGeometry(const OccupancyGrid& grid_map);
        void findContours();
        void trace(Contour& c, int row, int col, int prev_row, int prev_col);
        void trace( Contour& c, int row, int col, int prev_row, int prev_col, const Point<int>& start_pixel, std::optional<Point<int>> first_neighbor);
        void showContours(const cv::Mat& original_image);
        void listContours();

        void polygonize();
        std::vector<Segment<int>> traceCrackBoundary(const Contour& c);
        void showCrackBoundaries(const cv::Mat& original_image);

    private:
        std::vector<Contour> contours_;
        const OccupancyGrid& grid_map_; // immutable occupancy grid for reading only!
        std::vector<std::vector<int>> working_grid_; // copy of the grid map to use for suzuki-abe 
        std::vector<PolygonWithHoles> free_space_regions_;


        std::array<Point<int>, 8> clockwise_ {{
            { 1,  0},  // 0: E
            { 1,  1},  // 1: SE
            { 0,  1},  // 2: S
            {-1,  1},  // 3: SW
            {-1,  0},  // 4: W
            {-1, -1},  // 5: NW
            { 0, -1},  // 6: N
            { 1, -1},  // 7: NE
        }};


        // Crack-level geometry: ordered directed cell-edge segments.
        // Still in padded working-grid coordinates.
        std::vector<std::vector<Segment<int>>> crack_boundaries_;

};