#pragma once
#include "cleaner/occupancy_grid.hpp"
#include <array>
#include <vector>
#include <algorithm>
#include <iostream>
#include "opencv2/opencv.hpp"

/*
    This class is repsonsible to make some polygons out of the obstalces inside the occupancy grid map
*/

struct Contour{
    int id_;
    std::vector<Point<int>> points_;
    Contour* parent_;
    bool is_hole_;

};

class Polygon {
    public:
        Polygon(){}
        std::vector<Point<double>> points_;
};

class MapGeometry{
    public:
        MapGeometry(const OccupancyGrid& grid_map);
        void findContours();
        void trace(Contour& c, int row, int col, int prev_row, int prev_col, const Point<int>& start_pixel);
        void showContours(const cv::Mat& original_image);
    private:
        std::vector<Contour> contours_;
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



};