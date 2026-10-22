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


inline double distance(const Point<double>& A , const Point<double>& B) {
    Point<double> r = B - A;
    return std::sqrt(r.x_*r.x_ + r.y_*r.y_);
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

template <typename T>
inline PointState pointInPolygon(const Point<double> P, const Polygon<T>& polygon) {
    std::vector<Point<double>> intersections;
    int count = 0;
    for (int i = 0 ; i <polygon.points_.size() ; i++)
    {
        // Convert polygon coordinates to continuous geometry.
        Point<double> A{ static_cast<double>(polygon.points_.at(i).x_), static_cast<double>(polygon.points_.at(i).y_) };
        Point<double> B{ static_cast<double>( polygon.points_.at((i + 1) % polygon.points_.size()).x_), static_cast<double>( polygon.points_.at((i + 1) % polygon.points_.size()).y_) };

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
inline void normalize(Point<double>& dir) {
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


inline std::pair<double,double> getProjectionRange(const Polygon<double>& polygon, Point<double> direction) {
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
inline std::vector<Point<double>> inwardUnitNormalsForPolygon(const Polygon<double>& polygon) {
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
    =======================================================================

    There are 3separate issues:
    1. region survives as one polygon
    2. region splits into multiple polygons
    3. region disappears completely --> (narrow polygons disappear after a scale!

    Following function only naturally represents case 1.
*/
/**
    TODO: I NEED TO MAKE THE FOLLOWING MORE ROBUST so that it handles region elimination and other corner cases!
*/
inline Polygon<double> offsetEdgesForPolygon(const Polygon<double>& polygon, double rad, Offset offset) {
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



inline PolygonWithHoles<double> applyClearance(const PolygonWithHoles<double>& region, double radius) {
    PolygonWithHoles<double> safe_region_;
    safe_region_.outer_ = offsetEdgesForPolygon(region.outer_, radius, Offset::INWARD);
    for (const auto& hole : region.holes_){
        safe_region_.holes_.push_back(offsetEdgesForPolygon(hole, radius, Offset::OUTWARD));
    }
    return safe_region_;
}


/*
    discrete raster geometry
            |
            | ONE conversion
            v
    continuous planning geometry


    SO:
        OccupancyGrid
            ↓
        MapGeometry
            ↓
        PolygonWithHoles<int>
            grid vertices are exact integers

            ↓ toDoubleGeometry()       ← ONE-WAY BOUNDARY

        PolygonWithHoles<double>
            still measured in grid cells

            ↓ clearance
            ↓ BCD
            ↓ arbitrary-angle lanes
            ↓ trajectory

*/
inline Polygon<double> toDoublePolygon(const Polygon<int>& polygon) {
    Polygon<double>  result;
    result.points_.reserve(polygon.points_.size());
    for(const auto& p : polygon.points_) {
        result.points_.push_back({static_cast<double>(p.x_), static_cast<double>(p.y_)});
    }
    return result;
}

inline PolygonWithHoles<double> toDoubleGeometry(const PolygonWithHoles<int>& region) {
    PolygonWithHoles<double> result;
    result.outer_ = toDoublePolygon(region.outer_);
    for(const auto& hole : region.holes_) {
        result.holes_.push_back(toDoublePolygon(hole));
    }
    return  result;
}


/*
    range: (-π, π]

    0°    ->   0°
    180°  -> 180°
    -180° -> 180°
    270°  -> -90°
    -270° ->  90°

*/
inline void piToPi(double& theta) {
    theta = fmod(theta, 2*M_PI); // now theta is in (-2π, 2π)
    if (theta > M_PI) theta-= 2*M_PI;
    if (theta <= -M_PI) theta+= 2*M_PI;
}
// so 0 should stay 0, and π should become 0 in below function --> for lines (infinite segments) the line with PI angle is the same as 0
/*
examples : 
    0° ->   0°
    30° ->  30°
    150° -> 150°
    180° ->   0°
    210° ->  30°
    -30° -> 150°
    360° ->   0°

*/
inline void zeroToPi(double& theta) {
    theta = fmod(theta, M_PI);
    if (theta < 0) theta += M_PI;
}

/*
    cross (z, direction)
    These functions are assumed to be used to find a normal dir of a vector in xy plane!
*/
inline Point<double> normalLeft(const Point<double>& direction) {
    return Point<double>{-direction.y_ , direction.x_};
}
/*
    cross (direction, z)
*/
inline Point<double> normalRight(const Point<double>& direction) {
    return Point<double>{direction.y_ , -direction.x_};
}



/*
    Assuming line (infinte segment) is represented by P.n = c  --> so dot product of p and n is c. P is just a point in a line and n is the normal vector of that line and c is just offset!
    sanity check with n=(1,0). so if you expand the dot prodcut you get ax+by=c  assuming n=(a,b) and p=(x,y)
    polygon has different segements so X = A + t (B-A)
    and line is X.n = c 
    so (A + t(B-A)).n = c so you can find t easily! t = (c- A.n) / (B-A).n


    I remember I used some other represenation somewhere in the codebase (maybe in test_computational_geometry.cpp) that used an if check before finding the intersection but here we check if t is betweeen 0 and 1 and its enough


    so P.n = c is : all points whose projection onto n is the same distance c. so basically c is the min distance of that line to origin
*/
inline std::vector<Point<double>> linePolygonIntersections( const Polygon<double>& polygon, const Point<double>& unit_normal, double c) {
    std::vector<Point<double>> intersections_;

    const double eps = 1e-10;

    for (int i = 0 ; i < polygon.points_.size(); i++) {

        Point<double> A = polygon.points_.at(i);
        Point<double> B = polygon.points_.at((i+1)%polygon.points_.size());

        double den = dotProduct2D(B-A, unit_normal);

        // Polygon edge is parallel to the infinite line.
        if (std::abs(den) < eps) {

            double A_projection = dotProduct2D(A, unit_normal);

            // CASE 1:
            // Parallel, but on a different line -> no intersection.
            if (std::abs(A_projection - c) >= eps) {
                continue;
            }

            // CASE 2:
            // The whole polygon edge lies on the infinite line.
            //
            // There is not one unique intersection point here;
            // the intersection is actually the whole segment A-B.
            //
            // For now store both endpoints.
            // The cleaning function later removes duplicated points
            // caused by neighboring edges / shared vertices.
            intersections_.push_back(A);
            intersections_.push_back(B);

            continue;
        }

        double t =
            (c - dotProduct2D(A, unit_normal)) / den;

        // Small tolerance is used because floating-point calculations
        // may produce values such as -1e-15 or 1.000000000000001.
        if (t >= -eps && t <= 1.0 + eps) {

            // Clamp tiny numerical errors back onto the segment.
            if (t < 0)
                t = 0;

            if (t > 1)
                t = 1;

            // There is an intersection with the finite polygon edge.
            double x = A.x_ + t * (B.x_ - A.x_);
            double y = A.y_ + t * (B.y_ - A.y_);

            intersections_.push_back(
                Point<double>{x,y}
            );
        }
    }

    return intersections_;
}

/**
 * clipLineToPolygonIntersections()
 *
 * Goal:
 *     Take one infinite line and clip it against a possibly concave polygon.
 *     The result may contain zero, one, or multiple inside line segments.
 *
 *
 * 1. Get raw boundary intersections
 * ---------------------------------
 *     Call linePolygonIntersections().
 *
 *     The returned points are all detected contacts between the infinite
 *     line and polygon edges.
 *
 *     These raw contacts may include:
 *         - proper crossings of the polygon boundary
 *         - duplicate points when the line passes through a polygon vertex
 *         - tangent contacts where the line only touches the polygon
 *         - endpoints of a polygon edge that lies exactly on the line
 *
 *     Therefore, raw intersection points must NOT immediately be assumed
 *     to represent alternating inside/outside crossings.
 *
 *
 * 2. Sort intersections along the lane direction
 * -----------------------------------------------
 *     First obtain the lane direction from the line normal.
 *
 *     For every intersection point P, calculate:
 *
 *         projection = P . lane_direction
 *
 *     Sort using this value so that the points are ordered along
 *     the infinite line:
 *
 *         P0 ---- P1 ---- P2 ---- P3 ---- ...
 *
 *     Sorting by x or y alone is not sufficient because the lane may
 *     have an arbitrary angle.
 *
 *
 * 3. Clean duplicate intersections
 * --------------------------------
 *     After sorting, remove consecutive points that are approximately
 *     equal using an epsilon.
 *
 *     Duplicate points commonly occur when:
 *         - two polygon edges share a vertex lying on the line
 *         - a coincident polygon edge contributes its endpoints
 *         - floating-point calculations produce nearly identical points
 *
 *     Do NOT use exact floating-point equality.
 *
 *
 * 4. Handle trivial cases
 * -----------------------
 *
 *     0 intersections:
 *         the line does not intersect the polygon
 *         -> return no segments
 *
 *     1 intersection:
 *         usually a tangent / touching case
 *         -> there is no interval to classify
 *         -> return no inside segment
 *
 *     2 or more intersections:
 *         inspect every interval between consecutive intersection points.
 *
 *     More than two intersections is completely valid for a concave polygon.
 *
 *
 * 5. Classify every interval using its midpoint
 * ---------------------------------------------
 *     Do NOT blindly pair:
 *
 *         P0-P1, P2-P3, ...
 *
 *     because raw boundary contacts are not guaranteed to be proper
 *     inside/outside crossings.
 *
 *     Instead, for every consecutive pair:
 *
 *         Pi ---------------- Pi+1
 *
 *     calculate the midpoint:
 *
 *         M = (Pi + Pi+1) / 2
 *
 *     and classify M using pointInPolygon().
 *
 *
 *     If M is INSIDE:
 *
 *         the interval Pi -> Pi+1 lies inside the polygon
 *         -> keep Segment(Pi, Pi+1)
 *
 *
 *     If M is OUTSIDE:
 *
 *         the interval lies outside the polygon
 *         -> discard it
 *
 *
 *     If M is BOUNDARY:
 *
 *         this indicates a degenerate case, most commonly:
 *             - the line lies along a polygon edge
 *             - part of the interval is coincident with the polygon boundary
 *
 *         Handle this explicitly according to the desired coverage policy.
 *         Do not automatically treat it as an interior coverage segment.
 *
 *
 * 6. Return all inside intervals
 * ------------------------------
 *     A concave polygon may produce several separate inside portions
 *     along the same infinite line.
 *
 *     Example:
 *
 *              +---------+
 *              |         |
 *      +-------+         +-------+
 *      |                         |
 *  ----P0-----P1---------P2-----P3-----> lane direction
 *      |                         |
 *      +-------+         +-------+
 *              |         |
 *              +---------+
 *
 *     After midpoint classification:
 *
 *         midpoint(P0,P1) -> INSIDE
 *         midpoint(P1,P2) -> OUTSIDE
 *         midpoint(P2,P3) -> INSIDE
 *
 *     Therefore return:
 *
 *         Segment(P0, P1)
 *         Segment(P2, P3)
 *
 *
 * Important:
 * ----------
 *     The usual inside/outside alternating rule is valid only for proper
 *     boundary crossings after all tangent, duplicate, and coincident cases
 *     have been resolved correctly.
 *
 *     Since linePolygonIntersections() intentionally returns raw contacts,
 *     this function uses midpoint classification instead of relying only
 *     on crossing parity.
 *
 *     Therefore the return type must be:
 *
 *         std::vector<Segment<double>>
 *
 *     because one infinite line may create zero, one, or multiple
 *     valid inside lane segments.
 */
inline std::vector<Segment<double>> clipLineToPolygonIntersections(const Polygon<double>& polygon, const Point<double>& unit_normal, double c) {

    const double eps = 1e-10;
    std::vector<Segment<double>> inside_segments_;
    std::vector<Point<double>> intersections_ =  linePolygonIntersections(polygon, unit_normal, c);

    // From normal vector to lane direction:
    auto lane_direction = normalRight(unit_normal);

    std::sort(intersections_.begin(),intersections_.end(),[&](const Point<double>& A, const Point<double>& B){
        // Sort along the lane direction!
        /*
            Each point can be assumed to be a vector from origin to that point and getting a dot product of that vector to lane direction just projects the point in that direction
        */
        return dotProduct2D(A, lane_direction) < dotProduct2D(B, lane_direction);

    });
    intersections_.erase( std::unique( intersections_.begin(), intersections_.end(),
            [&](const Point<double>& A, const Point<double>& B) {
                return std::abs(A.x_ - B.x_) < eps
                    && std::abs(A.y_ - B.y_) < eps;
            }
        ),
        intersections_.end()
    );

    if (intersections_.size() < 2) {
        std::cout << "NO INSIDE SEGMENT \n";
        return std::vector<Segment<double>>();
    }

    std::cout << "CHECKING SEGMENTS INSIDE \n";

    for (size_t i = 0 ; i + 1 < intersections_.size() ; i++) {

        Point<double> start = intersections_.at(i);
        Point<double> end = intersections_.at(i+1);

        Point<double> sum = start + end;
        Point<double> mid{sum.x_/2.0, sum.y_/2.0};

        PointState state = pointInPolygon(mid, polygon);

        if (state == PointState::INSIDE) {

            inside_segments_.push_back(
                Segment<double>{start,end}
            );
        }
        else if (state == PointState::OUTSIDE) {

            // This interval lies outside the polygon.
            continue;
        }
        else if (state == PointState::BOUNDARY) {

            // Degenerate / coincident case:
            // this interval lies on the polygon boundary.
            // For now, do not treat it as an interior coverage segment.
            continue;
        }
    }
        return inside_segments_;
}



///////////////////////////////// 

/**
 * Possible geometric cases for two finite segments:
 *
 * 1. Non-parallel segments
 *    a) Proper crossing
 *       -> 1 intersection point
 *
 *    b) Touching at an endpoint
 *       -> 1 intersection point
 *
 *    c) Their infinite lines intersect, but the intersection lies
 *       outside one or both finite segments
 *       -> 0 intersection points
 *
 *
 * 2. Parallel but NOT collinear
 *    -> 0 intersection points
 *
 *
 * 3. Collinear
 *    a) Disjoint
 *       -> 0 intersection points
 *
 *    b) Touch at exactly one endpoint
 *       -> 1 intersection point
 *
 *    c) Overlap over a finite interval
 *       -> 2 points:
 *          the two endpoints of the overlapping portion
 *
 *
 * Return convention:
 *
 * 0 points -> no common point
 * 1 point  -> single-point intersection
 * 2 points -> collinear overlapping interval
 *
 *
 * Example of collinear overlap:
 *
 * Segment 1:   A ====================== B
 * Segment 2:          C ========== D
 *
 * Intersection:
 *                       C ========== D
 *
 * Returned points:
 *     C, D
 */
// The following is not based on orientationTest! that way (in the test_computational_geometry) was useful if we only wanted to return a bool of having intersection or not!
/*

    r × s != 0
        -> nonparallel
        -> calculate t,u
        -> actual intersection point if both lie in [0,1]

    r × s == 0
        |
        +-- q × r != 0
        |      -> parallel, separate
        |
        +-- q × r == 0
            -> collinear
            -> reduce to 1D
            -> disjoint / touching / overlap

*/
inline std::vector<Point<double>> segmentSegmentIntersection(const Segment<double> AB, const Segment<double> CD) {
    Point<double> A = AB.start_;
    Point<double> B = AB.end_;
    Point<double> C = CD.start_;
    Point<double> D = CD.end_;
    
    

    Point<double> r = B - A;
    Point<double> q = C - A;
    Point<double> s = D - C;

    double eps_ = 1e-10;
    if (dotProduct2D(r, r) <= eps_*eps_ ||
        dotProduct2D(s, s) <= eps_*eps_ ) {
            throw std::runtime_error("segmentSegmentIntersection: Segment has zero length!");
    }


    std::vector<Point<double>> intersection_; // Might be zero intersection , 1 intersection or an interval of intersections in collinear case where we return two end Points from the overlap!

    double den = crossProduct2D(r, s);
    if (std::abs(den) <= eps_ ){
        // Either Collinear or Parallel
        if(std::abs(crossProduct2D(q, r)) <= eps_) { // Read as cross(q,r)==0
            // Collinear! both segments exist on the same (infinite) line (By line I imply infinite but doesnt hurt to mention it)
            // Still need to check if they have an overlap or not! we dont know if C is near to the AB line or D so we check both ways
            double tc = dotProduct2D(C - A, r) / dotProduct2D(r, r);
            double td = dotProduct2D(D - A, r) / dotProduct2D(r, r);

            double lo = std::max(0.0, std::min(tc, td));
            double hi = std::min(1.0, std::max(tc, td));
            /*
                overall its either lo == hi which mean one point intersection which means end point touching
                or lo < hi significantly which means we are dealing with an interval!
            */
            if (hi > lo + eps_) {
                // then we are dealing with an overlap interval!
                Point<double> start_of_interval_ = A + r * lo;
                Point<double> end_of_interval_ = A + r * hi;
                intersection_.push_back(start_of_interval_);
                intersection_.push_back(end_of_interval_);
                return intersection_;
            }
            else if (std::abs(lo - hi) <= eps_) {
                // we are dealing with single point (end point touching) so if start and end are the same then we are in end point touching
                Point<double> point_ = A + r * lo;
                intersection_.push_back(point_);
                return intersection_;

            }
            // Otherwise hi < lo significantly:
            // collinear but disjoint
            return intersection_;
        }
        else {
            // Parallel but not collinear -> 0 intersections
            return intersection_;
        }
    }
    else {
        // Surely there is an Intersection assuming Infinite lines! but for segemetns intersections we gotta check! 
        double t = crossProduct2D(q, s) / den;
        double u = crossProduct2D(q, r) / den;
        if (t >= -eps_ && t <= 1 + eps_ && u >=-eps_ && u <= 1 + eps_) {
            /*
                And after accepting t with tolerance  
                Otherwise a legitimate endpoint intersection might produce something like:
                    t = -0.00000000003
                and your returned point would technically be microscopically outside segment AB.
            */
            t = std::clamp(t, 0.0, 1.0); //

            // surely there is an intersection between segments
            double x = A.x_ + t * (B.x_ - A.x_);
            double y = A.y_ + t * (B.y_ - A.y_);
            intersection_.push_back(Point<double>{x,y});
        }
    }


    /*
        infinite lines intersect
                ↓
        but t or u is outside [0,1]
    */
    return intersection_;


}


struct SegmentPolygonIntersection
{
    std::vector<Point<double>> points_;
    std::vector<Segment<double>> overlaps_;
};

inline SegmentPolygonIntersection segmentPolygonIntersections( const Segment<double>& segment, const Polygon<double>& polygon) {

    SegmentPolygonIntersection intersections_;
    for (int i = 0; i < polygon.points_.size(); i++) {
        Point<double> A = polygon.points_.at(i);
        Point<double> B = polygon.points_.at((i+1)%polygon.points_.size());
        
        auto intersection_ =  segmentSegmentIntersection(segment, Segment<double>{A, B});

        if (intersection_.size() == 0 ) {
            // No intersection
            continue;
        }
        else if (intersection_.size() == 1) {
            // One intersection
            intersections_.points_.push_back(intersection_.at(0));
        }
        else if (intersection_.size() == 2) {
            // Overlap 
            intersections_.overlaps_.push_back(Segment{intersection_.at(0), intersection_.at(1)});
        }
    }

    /*
        deduplicate
        points_ = unique isolated intersection points
        overlaps_ = unique overlap intervals

        overlaps_: --> But the thing is i already am doing a simplification of polygon representation so if a straight semgent is from P0 to P1 and P2 i already am removed the P1 out ofthe picture but it never hurts to be safe!
            P0 -> P1
            P1 -> P2
    */

    // sort them along the candidate segment
    std::sort(intersections_.points_.begin(), intersections_.points_.end() , [&](const Point<double>& A , const Point<double>& B){
        Point<double> r = segment.end_ - segment.start_; // Segment direction
        return dotProduct2D(A, r) < dotProduct2D(B, r);
    });

    // Beware that std::unique in its default form uses Point<double>::operator== but here we use epsilon safety otherwise two copies of the same polygon vertex such as (5.0, 3.0) and (5.00000000001, 3.0)
    intersections_.points_.erase(std::unique(intersections_.points_.begin(),intersections_.points_.end(),[](const Point<double>& A, const Point<double>& B){
        double eps_ = 1e-10;
        if (std::abs(A.x_ - B.x_) <= eps_ && std::abs(A.y_ - B.y_) <= eps_) return true;
        else return false;
    }), intersections_.points_.end());

    /*
        sort overlaps by start position along candidate segment

        if no overlaps:
            done

        current = first overlap

        for each remaining overlap:
            next = ...

            if next.start is at or before current.end (+epsilon):
                extend current.end to the farther of:
                    current.end
                    next.end

            else:
                save current
                current = next

        save final current


        A----B
            C----D
                E----F



        current: A---------------B
        next:          C----D
    */
    // sort them along the candidate segment --> maybe sorting each segment by its starting point is enough
    std::sort(intersections_.overlaps_.begin(), intersections_.overlaps_.end() , [&](const Segment<double>& AB , const Segment<double>& CD){
        Point<double> r = segment.end_ - segment.start_; // Segment direction

        return dotProduct2D(AB.start_, r) < dotProduct2D(CD.start_, r);
    });

    if (intersections_.overlaps_.size() > 1 ) {
        std::vector<Segment<double>> deduplicated_overlaps_;
        auto current_ = intersections_.overlaps_.at(0);
        for (int i = 1 ; i <intersections_.overlaps_.size() ; i++) {
            Point<double> A = current_.start_;
            Point<double> B = current_.end_;

            auto next_ = intersections_.overlaps_.at(i);
            Point<double> C = next_.start_;
            Point<double> D = next_.end_;
            Point<double> r = B - A;
            double tc = dotProduct2D(C - A, r) / dotProduct2D(r, r);
            double eps_ = 1e-10;
            if( tc <= 1 + eps_) {

                double td = dotProduct2D(D - A, r) / dotProduct2D(r, r);
                if (td > 1 + eps_){
                    current_.end_ = next_.end_;
                }
                else {
                    // its this case:
                    // current: A---------------B
                    // next:          C----D
                    // so the current end remains the same!
                }

            }
            else {
                // Gap -> current interval is finished
                deduplicated_overlaps_.push_back(current_);

                // Start a new interval
                current_ = next_;

            }

        }
        // Save the last active interval
        deduplicated_overlaps_.push_back(current_);

        intersections_.overlaps_ = deduplicated_overlaps_;
    }

    // remove the points that are already part of the overlap
    // For each point, ask: Does this point lie on any segment in overlaps_?
    double eps_ = 1e-10;

    intersections_.points_.erase(
        std::remove_if(
            intersections_.points_.begin(),
            intersections_.points_.end(),
            [&](const Point<double>& point_)
            {
                for (const auto& overlap_ : intersections_.overlaps_)
                {
                    Point<double> A = overlap_.start_;
                    Point<double> B = overlap_.end_;
                    Point<double> r = B - A;

                    double t =
                        dotProduct2D(point_ - A, r) /
                        dotProduct2D(r, r);

                    if (t >= -eps_ && t <= 1.0 + eps_)
                    {
                        return true; // remove this point
                    }
                }

                return false; // keep this point
            }),
        intersections_.points_.end()
    );



    return intersections_;
}



inline bool isSegmentInsidePolygon(const Segment<double>& segment, const Polygon<double>& polygon) {

    Point<double> A = segment.start_;
    Point<double> B = segment.end_;
    Point<double> r = B - A;
    double eps_ = 1e-10;
    if (dotProduct2D(r, r) <= eps_ * eps_) {
        throw std::runtime_error("isSegmentInsidePolygon: Segment Length is zero!");
    }

    auto s1 = pointInPolygon(A, polygon);
    auto s2 = pointInPolygon(B, polygon);

    if (s1 == PointState::OUTSIDE || s2 == PointState::OUTSIDE) {
        return false;
    }
    // So now both endpoint are either inside or boundary :  both endpoints being inside does not guarantee that the line between them stays inside so we do the mid point check
    auto intersections_ = segmentPolygonIntersections(segment, polygon);

    auto intersection_points_ = intersections_.points_;
    auto intersection_overlaps_ = intersections_.overlaps_;

    /*
        Treat the candidate segment as a line that you want to divide into smaller intervals. You need to divide it at every place where it touches or overlaps the polygon boundary.
        The procedure is:
        1. Start with the candidate endpoints, A and B.
        2. Add every isolated intersection point.
        3. Add the start and end of every overlap segment.
        4. Sort all these points in the order they occur along the candidate segment.
        5. Test the midpoint of each consecutive pair. If any midpoint is OUTSIDE, return false. Otherwise, return true.
    */
    std::vector<Point<double>> all_important_points_;
    all_important_points_.push_back(segment.start_);
    all_important_points_.push_back(segment.end_);
    for (const auto& p : intersection_points_) {
        all_important_points_.push_back(p);
    }
    for (const auto& s : intersection_overlaps_) {
        all_important_points_.push_back(s.start_);
        all_important_points_.push_back(s.end_);
    }

    std::sort(all_important_points_.begin(), all_important_points_.end(), [&](const Point<double>& A, const Point<double>& B){
        Point<double> seg_dir_ = segment.end_ - segment.start_;
        // How far along the segment direction does the position vector of this point project?
        return dotProduct2D(A, seg_dir_) < dotProduct2D(B, seg_dir_);
        // one could also use this : How far along the segment direction does the displacement from the segment start project?
        // dotProduct2D(P - segment.start_, seg_dir_)
        /*
            these dot products are projection scores, not the actual signed projection distances. To obtain the signed distance along the segment's direction, divide by \(\|\mathbf{d}\|\). To obtain the normalized segment parameter \(t\), divide by \(\mathbf{d}\cdot\mathbf{d}\):
                t = (P - S).d / d.d
        */
    });



    for (int i = 0 ; i + 1 < all_important_points_.size() ; i++) {
        Point<double> A = all_important_points_.at(i);
        Point<double> B = all_important_points_.at(i+1);
        Point<double> mid_point_ = Point<double>{(A.x_ + B.x_)/2 , (A.y_ + B.y_)/2};
        auto state = pointInPolygon(mid_point_ , polygon);
        if (state == PointState::OUTSIDE) {
            return false;
        }
    }


    return true;


}




inline std::vector<double> polygonSegmentAngles(const Polygon<double>& polygon) {
    std::vector<double> angles_;

    for (int i = 0 ; i < polygon.points_.size() ; i++) {
        Point<double> A = polygon.points_.at(i);
        Point<double> B = polygon.points_.at((i+1)%polygon.points_.size());

        double angle = std::atan2(B.y_ - A.y_ , B.x_ - A.x_); // radians, [-pi, pi]
        if (angle < 0.0) {
            angle += M_PI;
        }

        if (angle >= M_PI) {
            angle -= M_PI;
        }
        // Now angle is in [0, pi) for an undirected line orientation.
        angles_.push_back(angle);
    }
    // Deduplicate with a tolerance
    angles_.erase((std::unique(angles_.begin(),angles_.end(),[](double a, double b){
        // unique with an epsilon tolerance
        double eps_ = 1e-10;
        return std::abs(a-b)< eps_;
    })), angles_.end());

    return angles_;
}

inline std::vector<double> candidateLaneAnglesForPolygon(std::vector<double> angles, const Polygon<double>& polygon) {
    // if any of the lane directions produces more than two segments then it means one line is producing more than two intersections and its not in accord with polygon's monotonicity! so we discard it
}



/*
 * Returns whether a simple polygon can be covered using parallel lanes
 * at the given lane direction, such that every lane intersects the
 * polygon in at most one connected segment.
 *
 * Assumptions:
 * - The polygon is simple and has no holes.
 * - angle is the lane direction, in radians.
 *
 * Implementation steps:
 *
 * 1. Validate the polygon.
 *    - If it has fewer than three vertices, return false.
 *
 * 2. Calculate the unit normal perpendicular to the lane direction.
 *    - nx = -sin(angle)
 *    - ny =  cos(angle)
 *
 * 3. Project every polygon vertex onto this normal.
 *    - For each vertex p[i], calculate:
 *          s[i] = dot(p[i], normal)
 *    - These values describe the position of each vertex across
 *      the parallel lanes.
 *
 * 4. Find the minimum and maximum projection values and their
 *    corresponding vertex indices.
 *    - These identify the bottommost and topmost parts of the
 *      polygon in the coordinate system of this sweep direction.
 *
 * 5. Check the polygon boundary in both directions.
 *    - Starting at a minimum-projection vertex, follow the polygon
 *      vertices in their original cyclic order until reaching a
 *      maximum-projection vertex.
 *    - Check that the projection values never decrease, allowing
 *      a small numerical tolerance.
 *    - Repeat the check while traversing the polygon in the
 *      opposite direction.
 *
 * 6. Decide whether the direction is sweepable.
 *    - If both boundary chains are nondecreasing, return true.
 *    - If either chain decreases, return false.
 *
 * Important:
 * - Do not generate lanes or use lane spacing in this function.
 * - Equal projections are allowed, including those caused by
 *   edges parallel to the lanes.
 * - Handle repeated minimum/maximum projections and collinear
 *   boundary edges carefully.
 * - Use the projection tolerance consistently when comparing values.
 */
inline bool isSweepable( const Polygon<double>& polygon, double angle) {
    if (polygon.points_.size() < 3) 
        throw std::runtime_error("isSweepable : Polygon has less than 3 verticies");

    Point<double> lane_direction_{std::cos(angle), std::sin(angle)};
    Point<double> lane_normal_ = normalLeft(lane_direction_); // {-sin(angle), cos(angle)}

    std::vector<double> projections_; // Aligned with polygon.points_ indices

    for (int i = 0 ; i < polygon.points_.size() ; i++) {
        projections_.push_back(dotProduct2D(polygon.points_.at(i), lane_normal_));
    }

    auto min_it_ = std::min_element(projections_.begin(), projections_.end());
    auto max_it_ = std::max_element(projections_.begin(), projections_.end());
    
    size_t min_index_ = std::distance(projections_.begin(), min_it_);
    size_t max_index_ = std::distance(projections_.begin(), max_it_);

    // forward move
    for (size_t i = min_index_ ; i + 1 < max_index_ ; i++) {
        if (projections_.at(i) > projections_.at(i+1))
            return false;
    }

    // Backward move
    for (size_t i = 0 ; i < projections_.size() ; i++) {
        

    }


    
}