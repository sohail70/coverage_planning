#pragma once
#include "cleaner/occupancy_grid.hpp"
#include <array>
#include <opencv2/core/traits.hpp>
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
 *   - obstacle holes inside free space --> holes are the ones enclosed by the freespace (basically black region)


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






struct BoundaryGeometry
{
    /*
     * Identity inherited from the Suzuki contour.
     *
     * id_ and parent_id_ preserve the hierarchy so later stages
     * do not need to search back through contours_.
     */
    int id_;
    int parent_id_;

    /*
     * false -> outer boundary of a free-space component
     * true  -> hole boundary inside free space
     */
    bool is_hole_;


    /*
     * Exact raster crack geometry.
     *
     * Ordered directed unit grid edges.
     * Still in padded working-grid coordinates.
     */
    std::vector<Segment<int>> crack_boundary_;


    /*
     * Simplified polygon ring derived from crack_boundary_.
     *
     * Straight runs have been compressed to their corner vertices.
     */
    Polygon<int> polygon_;
};

template<typename T>
struct FreeSpaceRegion{
    PolygonWithHoles<T> geometry_;
    int outer_boundary_id_;

    bool reachable_ = false; //reachable_ = reachable from the robot without crossing an obstacle, according to static connected-component topology
};

class MapGeometry{
    public:
        MapGeometry(const OccupancyGrid& grid_map);
        void findContours();
        void trace(Contour& c, int row, int col, int prev_row, int prev_col);
        void trace( Contour& c, int row, int col, int prev_row, int prev_col, const Point<int>& start_pixel, std::optional<Point<int>> first_neighbor);
        void showContours(const cv::Mat& original_image);
        void showNBD(int x0, int y0, int width, int height);

        void listContours();

        void polygonize();
        std::vector<Segment<int>> traceCrackBoundary(const Contour& c);
        void showCrackBoundaries(const cv::Mat& original_image);
        Polygon<int> makePolygonFromBoundary(const std::vector<Segment<int>>& boundary);
        void showSimplifiedBoundaries(const cv::Mat& original_image);
        void listSimplifiedContours();


        void determineReachableRegion(const Point<double>& robot_position);

    private:
        const OccupancyGrid& grid_map_; // immutable occupancy grid for reading only!
        std::vector<std::vector<int>> working_grid_; // copy of the grid map to use for suzuki-abe 



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





        // Suzuki result.
        std::vector<Contour> contours_;

        // Derived polygon geometry, while preserving Suzuki hierarchy.
        std::vector<BoundaryGeometry> boundary_geometries_;

        // Final regions that will be sent to BCD.
        std::vector<FreeSpaceRegion<int>> free_space_regions_;



        /*
        converting:

            1 root
            2 outer
            ├── 3 hole
            │   └── 5 outer
            └── 4 hole

        TO:

            free_space_regions_[0]
                outer = polygon of 2
                holes = polygon of 3
                        polygon of 4

            free_space_regions_[1]
                outer = polygon of 5
                holes = none
        
        */
        void buildFreeSpaceRegions();


};