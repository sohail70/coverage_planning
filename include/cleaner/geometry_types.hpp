#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <math.h>
#include <ostream>
#include <stdexcept>
#include <vector>
#include <iostream>
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



enum class PointState {
    INSIDE,
    OUTSIDE,
    BOUNDARY,
};


struct SegmentPolygonIntersection
{
    std::vector<Point<double>> points_;
    std::vector<Segment<double>> overlaps_;
};

enum class Offset {
    OUTWARD,
    INWARD,
};


enum class Orientation {
    CW,
    CCW,
    COLLINEAR,
};