#pragma once
#include "cleaner/geometry.hpp"


enum class CellState{
    Free,
    Occupied,
    Unknown,
};



// What the planner needs to know?
class OccupancyGrid{
    public:
        virtual ~OccupancyGrid() = default;


        /*
            Returns whether (row, col) identifies a valid cell in the occupancy grid.
            Grid indices are zero-based:
               row: [0, getHeight() - 1]
               col: [0, getWidth()  - 1]
            ex: height_ = 100  ----valid-rows----> 0,1,2,...,99 --> so we must have row < height_
        */
        bool inBounds(int row, int col) const {
            return row>=0 &&
                row< getHeight() &&
                col >= 0 &&
                col < getWidth();
        }



        virtual int getWidth() const = 0;
        virtual int getHeight() const = 0;

        

        virtual double getResolution() const = 0;

        virtual CellState getCellState(int row, int col) const = 0;

        virtual Point<double> cellCenterToWorld(int row, int col) const = 0;
    private:

};