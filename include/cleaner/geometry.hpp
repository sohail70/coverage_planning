#pragma once
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
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


inline void piToPi(double& theta){
    theta = fmod(theta, 2*M_PI);
    if (theta > M_PI) theta-= 2*M_PI;
    if (theta <= -M_PI) theta+= 2*M_PI;
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