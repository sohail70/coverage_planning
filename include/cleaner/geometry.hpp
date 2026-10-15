#pragma once
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <math.h>
#include <ostream>
#include <stdexcept>
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





template<typename T>
Polygon<T> simplifyPolygon(const Polygon<T>& polygon)
{
    Polygon<T> result;

    if (polygon.points_.size() < 3)
        return polygon;


    // Phase 0: remove consecutive duplicates
    std::vector<Point<T>> points;

    for (const auto& p : polygon.points_)
    {
        if (points.empty() || points.back() != p)
        {
            points.push_back(p);
        }
    }

    // Our polygon representation does not need:
    // first point == last point
    if (points.size() > 1 &&
        points.front() == points.back())
    {
        points.pop_back();
    }


    if (points.size() < 3)
        return Polygon<T>{};


    // Remove collinear intermediate points
    // until stable
    bool changed = true;

    while (changed && points.size() >= 3)
    {
        changed = false;

        for (std::size_t i = 0; i < points.size(); ++i)
        {
            std::size_t prev =
                (i + points.size() - 1) % points.size();

            std::size_t next =
                (i + 1) % points.size();

            Point<T> A = points.at(prev);
            Point<T> B = points.at(i);
            Point<T> C = points.at(next);

            Point<T> u = B - A;
            Point<T> v = C - B;

            if (crossProduct2D(u, v) == 0 &&
                dotProduct2D(u, v) > 0)
            {
                points.erase(points.begin() + i);

                changed = true;
                break;
            }
        }
    }


    result.points_ = std::move(points);

    return result;
}




// make a vector into unit vector
void normalize(Point<double>& dir) {
    if(dir.x_==0 && dir.y_==0) {
        throw std::runtime_error("Normalize: zero dir!");
    }
    auto den = std::sqrt(dir.x_*dir.x_ + dir.y_*dir.y_);
    dir.x_ = dir.x_/den;
    dir.y_ = dir.y_/den;
}

/**
 * Orientation convention:
 *
 * We use the mathematical orientation determined directly from
 * the coordinate values and cross-product sign:
 *
 *      signed area > 0  -> CCW
 *      signed area < 0  -> CW
 *
 * Our current grid/image coordinates have:
 *
 *      +x -> right
 *      +y -> down
 *
 * Therefore a mathematically CCW polygon appears visually clockwise
 * when displayed as an image, and vice versa.
 *
 * Throughout the geometry code, Orientation::CW / CCW refer to the
 * mathematical/raw-coordinate convention, NOT the visual image orientation.
 */
template <typename T>
double signedPolygonArea(const Polygon<T>& polygon) {
    double areaTimesTwo = 0;
    for (int i = 0 ; i < polygon.points_.size(); i++) {
        auto p1 = polygon.points_.at(i);
        auto p2 = polygon.points_.at((i+1)%polygon.points_.size());
        auto cross = crossProduct2D(p1, p2);
        areaTimesTwo += cross;
    }
    return areaTimesTwo/2;
}

template <typename T>
Orientation polygonOrientation(const Polygon<T>& polygon) {
    if(polygon.points_.size() < 3){
        throw std::runtime_error("polygonOrientation: Polygon has less than 3 points\n");
    }
    auto signedArea = signedPolygonArea(polygon);
    if (signedArea > 0) 
        return Orientation::CCW;
    else if (signedArea < 0)
        return Orientation::CW;

    return Orientation::COLLINEAR; // degenerate case
}


std::pair<double,double> getProjectionRange(const Polygon<double>& polygon, Point<double> direction) {
    if (polygon.points_.empty())
        throw std::runtime_error("getProjectionRange: Empty polygon");

    normalize(direction);
    double first_projection = dotProduct2D(polygon.points_.at(0), direction);
    double min_ = first_projection;
    double max_ = first_projection;

    for (std::size_t i = 1; i < polygon.points_.size(); ++i) {
        double current_projection = dotProduct2D(polygon.points_.at(i), direction);
        if (current_projection > max_)
            max_ = current_projection;

        if (current_projection < min_)
            min_ = current_projection;
    }
    return {min_,max_};
}


/*
    linePolygonIntersections
    segmentIntersection
    simplifyPolygon
    offsetPolygon   

*/



/**
 * The polygon consists of directed edges, so first determine its orientation.
 *
 * Our stored coordinate system is:
 *     +x right
 *     +y down
 *
 * However, for Orientation::CCW / CW we keep the usual algebraic naming
 * convention associated with the signed-area / cross-product sign:
 *
 *     signed area > 0 -> CCW
 *     signed area < 0 -> CW
 *
 * Because the stored coordinates have +y down, these orientation names may
 * appear visually reversed when the polygon is displayed as an image.
 *
 * Therefore:
 *
 *     polygonOrientation() == CCW
 *         -> interior is on the algebraic "left" of each directed edge
 *         -> inward normal = z × v = (-dy, dx)
 *
 *     polygonOrientation() == CW
 *         -> interior is on the algebraic "right" of each directed edge
 *         -> inward normal = v × z = (dy, -dx)
 *
 * Here "left" and "right" are algebraic, not necessarily visual image left/right.
 *
 * This function returns one inward unit normal for each polygon edge.
 */
std::vector<Point<double>> inwardUnitNormalsForPolygon(const Polygon<double>& polygon) {
    Orientation orientation_ = polygonOrientation(polygon);
    std::vector<Point<double>> unit_normals_;
    for(int i = 0; i < polygon.points_.size() ; i++) {
        Point<double> A = polygon.points_.at(i);
        Point<double> B = polygon.points_.at((i+1)%polygon.points_.size());
        Point<double> dir = B - A;

        if (orientation_ == Orientation::CCW) {
            Point<double> normal{-dir.y_, dir.x_};
            normalize(normal);
            unit_normals_.push_back(normal);
        }
        else if (orientation_ == Orientation::CW) {
            Point<double> normal{dir.y_, -dir.x_};
            normalize(normal);
            unit_normals_.push_back(normal);
        }
        else {
            throw std::runtime_error("Degenerate case");
        }
    }

    return unit_normals_;

}



enum class Offset {
    OUTWARD,
    INWARD,
};

// shift the edges by r distance
/*

    For each edge A -> B:
    1. Shift A and B inward by rad along the edge's inward unit normal.
         A' = A + rad * n
         B' = B + rad * n

        For INWARD, use the inward unit normal.
        For OUTWARD, reverse the inward unit normal.

    2. The shifted points define an offset line.
    3. Intersect each pair of neighboring offset lines.
    4. Their intersection becomes the new vertex of the safe polygon.
       Line intersection:
         t = cross(C-A, s) / cross(r, s)
         P = A + t*r

    Treat offset edges as infinite lines when constructing the new vertices.
    We only need their intersection; t and u do not need to be in [0,1].
    This avoids treating the offset operation as a finite-segment intersection.
    TO UNDERSTAND IT YOU CAN PICTURE A SIMPLE CONCAVE POLYGON
*/
Polygon<double> offsetEdgesForPolygon(const Polygon<double>& polygon, double rad, Offset offset) {
    if (rad <= 0) {
        throw std::runtime_error("Offset distance is invalid");
    }
    Polygon<double> scaled_poly_;

    std::vector<Point<double>> unit_normals_ = inwardUnitNormalsForPolygon(polygon);

    std::vector<Segment<double>> segs_;
    for (int i = 0 ; i < polygon.points_.size() ; i++) {
        auto unit_normal_ = unit_normals_.at(i);
        if (offset == Offset::OUTWARD)
            unit_normal_ = unit_normal_ * -1;

        Point<double> A = polygon.points_.at(i);
        Point<double> B = polygon.points_.at((i+1)%polygon.points_.size());
        Point<double> A_prime = A + unit_normal_ * rad;
        Point<double> B_prime = B + unit_normal_ * rad;
        segs_.push_back({A_prime, B_prime});
    }

    // Now find the intersections between consecutive segs_ 
    // but assume they are line not segments because the segments 
    // are now shifted and we might have to extend them to find intersections. 
    // TO UNDERSTAND IT YOU CAN PICTURE A SIMPLE CONCAVE POLYGON
    for (int i = 0 ; i < segs_.size() ; i++) {
        std::size_t previous = (i + segs_.size() - 1) % segs_.size();

        Segment<double> AB = segs_.at(previous);
        Segment<double> CD = segs_.at(i);
        Point<double> A = AB.start_;
        Point<double> B = AB.end_;
        Point<double> C = CD.start_;
        Point<double> D = CD.end_;
        /*
            X = A + t*(B-A)
            Y = C + u*(D-C)

            X = Y is intersection --> A + t(B-A) = C + u(D-C) --> cross the both side with r and then s to find the t and u
            and since this is a line (infinte segment) then t and u is not necesserily between [0,1]
        */
        Point<double> r = B - A;
        Point<double> q = C - A;
        Point<double> s = D - C;
        
        auto den = crossProduct2D(r, s);
        if (std::abs(den) < 1e-12) {
            throw std::runtime_error(
                "offsetEdgesForPolygon: neighboring offset lines are parallel"
            );
        }

        double t = crossProduct2D(q, s) / den;
        double u = crossProduct2D(q, r) / den;

        auto intersection_point_ = A + r*t;
        scaled_poly_.points_.push_back(intersection_point_);
    }

    return scaled_poly_;
}