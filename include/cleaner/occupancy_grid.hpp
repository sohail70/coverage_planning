#pragma once
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <math.h>
#include <ostream>
#include <vector>

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
    bool operator!=(const Point& A) const{
        return (A.x_!=x_ ||  A.y_!=y_);
    }

    friend std::ostream& operator<<(std::ostream& out , const Point<T>& p){
        out<<"x: "<<p.x_<<", "<<"y: "<<p.y_<<"\n";
        return out;
    }

};
template <typename T>
struct Segment{
    Segment(){}
    Segment(Point<T> start, Point<T> end):start_(start),end_(end){}
    Point<T> start_;
    Point<T> end_;

    void reverse(){
        std::swap(start_, end_);
    }


    friend std::ostream& operator<<(std::ostream& out , const Segment<T> seg_){
        out<<"start_: "<<seg_.start_ <<"end_: "<<seg_.end_<<"\n";
        return out;
    }
};


// Geometry Level Data
template<typename T>
class Polygon {
public:
    std::vector<Point<T>> points_;
};

template<typename T>
struct PolygonWithHoles {
    Polygon<T> outer_;
    std::vector<Polygon<T>> holes_;
};





/*
    A cross/dot product is between vectors and vectors dont have specific position but are just directions!
    segment 1: A ----> B
    segment 2:          C ----> D

    u = B - A
    v = D - C
    cross(u, v)

*/
template<typename T>
T crossProduct2D(const Point<T>& A , const Point<T>& B){
    return (A.x_*B.y_ - A.y_*B.x_);
}
template<typename T>
T dotProduct2D(const Point<T>& A , const Point<T>& B){
    return (A.x_*B.x_ + A.y_*B.y_);
}

enum class Orientation {
    CW,
    CCW,
    COLLINEAR,
};

/*
    is c to the left or right ofthe directed line from a to b? To answer that, we need two vectors that start from the same origin.

    Image coordinates currently have +y downward, because they come from the image/grid. 
    Therefore the names CW and CCW from the cross-product sign are visually reversed compared with normal Cartesian coordinates.
    So be careful about the semantics

*/
template<typename T>
Orientation orientationTest(const Point<T>& a , const Point<T>& b , const Point<T>& c) {
    Point<T> AB = b - a;
    Point<T> AC = c - a;
    T cross = crossProduct2D(AB, AC);
    if(cross < 0)
        return Orientation::CW;
    else if(cross > 0)
        return Orientation::CCW;    
    else
        return Orientation::COLLINEAR;

}

inline Orientation orientationTest( const Point<int>& A, const Point<int>& B, const Point<double>& C) {
    double AB_x = static_cast<double>(B.x_ - A.x_);
    double AB_y = static_cast<double>(B.y_ - A.y_);
    double AC_x = C.x_ - static_cast<double>(A.x_);
    double AC_y = C.y_ - static_cast<double>(A.y_);
    double cross =
        AB_x * AC_y -
        AB_y * AC_x;
    if (cross < 0.0)
        return Orientation::CW;
    if (cross > 0.0)
        return Orientation::CCW;
    return Orientation::COLLINEAR;
}


enum class PointState {
    INSIDE,
    OUTSIDE,
    BOUNDARY,
};

inline PointState pointInPolygon(const Point<double> P, const Polygon<int>& polygon) {
    std::vector<Point<double>> intersections;
    int count = 0;
    for (int i = 0 ; i <polygon.points_.size() ; i++)
    {
        Point<int>  A = polygon.points_.at(i);
        Point<int>  B = polygon.points_.at((i+1)%polygon.points_.size());

        if (orientationTest(A, B, P) == Orientation::COLLINEAR){
            // Bounding box
            if(P.x_ >= std::min(A.x_,B.x_) &&
               P.x_ <= std::max(A.x_,B.x_) &&
               P.y_ >= std::min(A.y_ , B.y_) &&
               P.y_ <= std::max(A.y_ , B.y_)
                ){

                return PointState::BOUNDARY;
            }
        }

        if ((P.y_ < B.y_) != (P.y_ < A.y_)){
            double t = (P.y_ - A.y_) / (B.y_ - A.y_);
            double xIntersection = A.x_ + t * (B.x_ - A.x_);
            if(xIntersection > P.x_){
                intersections.push_back({xIntersection , P.y_});
                count++;
            }
        }
    }

    if ((count%2)==0){
        return PointState::OUTSIDE;
    }
    else if((count%2)!=0){
        return PointState::INSIDE;
    }

}

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