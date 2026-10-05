#pragma once
#include <cmath>
#include <math.h>

/*
    Point<int> for image --> basically x_ is horizontal so its column and y_ is vertical so it means rows in a image!
    Point<double> for world coordinate 
*/
template <typename T>
struct Point{
    Point(){}
    Point(T x, T y):x_(x),y_(y){

    }
    T x_;
    T y_;

    Point<T> operator+(const Point& A) const{
        return Point(x_ + A.x_, y_ + A.y_);
    }
    Point<T> operator-(const Point& A) const{
        return Point(x_ - A.x_ , y_ - A.y_);
    }
    Point<T> operator*(T t) const{
        return Point(x_*t , y_*t);
    }

    // Use in std::find() in map_geometry class trace function
    bool operator==(const Point& A) const{
        return (A.x_==x_&& A.y_==y_);
    }

};



/*

pB=R(θ)pA+t
you are choosing to define \(R(\theta)\) as the rotation that converts A coordinates into B coordinates.

                Physical point P
                       ●
                      / \
                     /   \
                    /     \
                 pA         pB
              local       world

transform() says:

"Given pA, tell me pB."

inverseTransform() says:

"Given pB, tell me pA."


*/
/*
    representation of point p in the translate/theta coordinate frame!
    pB​=R(θ)(pA)​+t

    input is pA and output is pB
*/
inline Point<double> transform(Point<double> p , double theta , Point<double> translate){
    return translate + Point<double>(std::cos(theta)*p.x_ - std::sin(theta)*p.y_ ,std::sin(theta)*p.x_ + std::cos(theta)*p.y_);
}
/*

    pB​−t=R(θ)pA​ -->  use inv(R(θ)) or transpose of it!
    input  here is pB and output is pA
*/

inline Point<double> inverseTransform(Point<double> p , double theta, Point<double> translate)
{
    Point<double> X = p - translate;
    return Point<double>{std::cos(theta)*X.x_ + std::sin(theta)*X.y_ , -std::sin(theta)*X.x_ + std::cos(theta)*X.y_}  ;
}


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