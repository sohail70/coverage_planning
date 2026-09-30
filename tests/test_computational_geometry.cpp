#include<iostream>
#include<vector>
#include<map>
#include<math.h>
#include<algorithm>


// I use the concept of Point as a vector if there is subtraction and addition happened to them so if we do addition/subtraction then see it conceptually as mathematical vector

struct Point{
    Point(double a, double b):x(a),y(b){};
    double x;
    double y;
    Point operator-(Point a){
        return  Point(x-a.x , y-a.y);
    }
    Point operator+(Point a){
        return Point(x+a.x, y+a.y);
    }
    Point operator*(double scalar){
        return Point(scalar*x,scalar*y);
    }


    void print(){
        std::cout<<"x: "<<x <<" y: "<<y<<"\n";
    }
};

// imagine a and b are vectors!
double crossProduct2D(Point a , Point b){
    return a.x*b.y - a.y*b.x;
}

double dotProduct2D(Point a , Point b){
    return a.x*b.x + a.y*b.y;
}

enum class Type{
    COVERAGE,
    TRANSITION,
};

struct Segment{
    Segment(Point p1 , Point p2):start(p1),end(p2){}
    Point start;
    Point end;
    Type type_;

    void reverseSeg(){
        Point temp = start;
        start = end;
        end = temp;
    }

};

std::map<int, std::string> OrientationLog;
enum class Orientation {
    CW,
    CCW,
    COLLINEAR,
};

/*
    is c to the left or right ofthe directed line from a to b? To answer that, we need two vectors that start from the same origin.

*/
Orientation orientationTest(Point a, Point b , Point c){
    Point ab = b - a;
    Point ac = c - a;
    double cp = crossProduct2D(ab,ac);
    if (cp < 0 ){
        return Orientation::CW;
    }
    else if (cp > 0) {
        return Orientation::CCW;
    }
    else{
        return Orientation::COLLINEAR;
    }
}


/*
    orientationTest(a, b, c)
    orientationTest(a, b, d)

    orientationTest(c, d, a)
    orientationTest(c, d, b)

    Why?

        For segment a → b, we ask:

            Where are c and d relative to the line a → b?

        And then for segment c → d:

            Where are a and b relative to the line c → d?


                c
                 \
                  \
        a ---------X--------- b
                    \
                     \
                      d



        For an intersection, c and d need to lie on opposite sides of the line a → b.

        Likewise, a and b need to lie on opposite sides of c → d.



        1. The side test: do they cross?
        Draw the infinite line through A and B. That line cuts the 2D plane into two sides.

        Now look at the endpoints C and D of the second stick.

        If C and D are on the same side of line AB, then the whole segment CD stays on that side. It cannot cross AB. No intersection.

        If C and D are on opposite sides, then segment CD must cross the infinite line AB somewhere.

        But that alone is not enough. The crossing might happen outside the finite part AB. So you also do the reverse test:

        Look at A and B relative to the infinite line through C and D.

        If A and B are on opposite sides, then segment AB crosses the infinite line CD.

        If both tests pass:

        CD crosses the infinite line AB

        AB crosses the infinite line CD

        then the two finite segments really do intersect.

        3. Where is the intersection point?
        Once you know they intersect, imagine walking along segment AB.

        You can describe any point on AB as:

        text
        P = A + t*(B - A)
        t = 0 means you are at A

        t = 1 means you are at B

        t = 0.5 means halfway

        Similarly, any point on CD is:

        text
        P = C + u*(D - C)
        u = 0 means you are at C

        u = 1 means you are at D

        Intersection means the same point P is on both segments:

        text
        A + t*(B - A) = C + u*(D - C)
        Solve that for t and u. If both are between 0 and 1, the intersection point lies on both finite segments.

        Then plug t back into:

        text
        x = A.x + t*(B.x - A.x)
        y = A.y + t*(B.y - A.y)
        That gives the intersection (x, y).




*/
bool segmantIntersection(Segment s1, Segment s2){
    Point a = s1.start;
    Point b = s1.end;
    Point c = s2.start;
    Point d = s2.end;

    Orientation o1 = orientationTest(a,b,c);
    Orientation o2 = orientationTest(a,b,d);
    Orientation o3 = orientationTest(c,d,a);
    Orientation o4 = orientationTest(c,d,b);
    std::cout<<static_cast<int>(o1)
                <<static_cast<int>(o2)
                <<static_cast<int>(o3)
                <<static_cast<int>(o4)<<"\n";

    // c and d are in the opposite side of ab line
    bool c1 = ((o1==Orientation::CW && o2==Orientation::CCW || o1==Orientation::CCW && o2==Orientation::CW));
    bool c2 = ((o3==Orientation::CW && o4==Orientation::CCW || o3==Orientation::CCW && o4==Orientation::CW));
    // Normal Case (non colinear)
    if(c1 && c2)
    {
        // computeIntersection
        Point r = b - a;
        Point q = c - a;
        Point s = d - c;
        double denominator = crossProduct2D(r,s);
        if (denominator != 0){ // It surely isnt 0 or else it means the lines would be parallel and c1 and c2 woulda caught it
            double t = crossProduct2D(q,s) / denominator;
            double u = crossProduct2D(q,r) / denominator;
            if (t>0 && t<1 && u>0 && u<1){ // REDUNDANT CHECK!
                Point intersection_point {a.x + t * (b-a).x , a.y + t*(b-a).y};
                intersection_point.print();
            }
        }



        return true; // surely there is an intersection!
    }


    /*
        Collinear overlap:
        
        Reduce the 2D problem to a 1D interval problem using AB as the reference.

        Any point P on the line AB can be written as:
            P = A + t * r
        where:
            r = B - A
            t = 0 -> A
            t = 1 -> B

        To find the position of C along AB, set P = C:
            C = A + tC * r
            C - A = tC * r

        Take the dot product with r on both sides:
            dot(C - A, r) = tC * dot(r, r)

        Therefore:
            tC = dot(C - A, r) / dot(r, r)

        Similarly:
            tD = dot(D - A, r) / dot(r, r)

        The dot product is used to project C-A and D-A onto the
        direction of AB, giving their 1D positions along the line.

        AB corresponds to [0, 1].
        CD corresponds to [min(tC,tD), max(tC,tD)].

        Overlap: --> vase fahm in bayad farz kuni to one D hasti! chun by using AB as a reference you are in 1D now! 
            lo = max(0, min(tC,tD))
            hi = min(1, max(tC,tD))

        If lo <= hi, the segments overlap.

        Convert the overlap range back to 2D:
            overlapStart = A + lo * r
            overlapEnd   = A + hi * r
    */

    if (o1 == Orientation::COLLINEAR &&
        o2 == Orientation::COLLINEAR &&
        o3 == Orientation::COLLINEAR &&
        o4 == Orientation::COLLINEAR)
    {
        Point r = b - a;
        double tc = dotProduct2D(c-a,r)/dotProduct2D(r,r);
        double td = dotProduct2D(d-a,r)/dotProduct2D(r,r);
        double lo = std::max(0.0 , std::min(tc,td));
        double hi = std::min(1.0,std::max(tc,td));
        if(lo <= hi){
            // std::cout<<"RANGE OF THE OVERLAPPING REGION IN 1D with AB ref:"<<lo <<" to "<<hi<<"\n";
            Point start = a + r*lo;
            Point end = a + r*hi;
            std::cout<<"RANGE OF OVERLAP FROM:"<<"\n";
            start.print();
            std::cout<<"TO: ";
            end.print();
            return true;
        }
    }






    //intersection on the start/end point of the vectors --> you need to return the point it self if you want to have the intersection point!
    // if c on segment ab --> but it doenst mean necessrilly its on the finite line!--> we need another test inside of it!
    /*
    orientation == COLLINEAR
            ↓
    C is on infinite line AB
            ↓
    bounding-box test
            ↓
    C is on finite segment AB
    */
   
    if (o1 == Orientation::COLLINEAR )
    {
        // bounding box test
        if (c.x >= std::min(a.x,b.x) && c.x <= std::max(a.x,b.x)
            && c.y>= std::min(a.y,b.y)&& c.y<=std::max(a.y,b.y))
        {
            // intersection point is the point c it self!
            c.print();

            return true;
        }
    }
    // if d on segment ab
    if (o2 == Orientation::COLLINEAR )
    {
        // bounding box test
        if (d.x >= std::min(a.x,b.x) && d.x <= std::max(a.x,b.x)
            && d.y>= std::min(a.y,b.y)&& d.y<=std::max(a.y,b.y))
        {
            d.print();
            return true;
        }

    }

    // if a on segment cd
    if (o3 == Orientation::COLLINEAR )
    {
        // bounding box test
        if (a.x >= std::min(c.x,d.x) && a.x <= std::max(c.x,d.x)
            && a.y>= std::min(c.y,d.y)&& a.y<=std::max(c.y,d.y))
        {
            a.print();
            return true;
        }

    }

    // if b on segment cd
    if (o4 == Orientation::COLLINEAR )
    {
        // bounding box test
        if (b.x >= std::min(c.x,d.x) && b.x <= std::max(c.x,d.x)
            && b.y>= std::min(c.y,d.y)&& b.y<=std::max(c.y,d.y))
        {
            b.print();
            return true;
        }

    }



    return false;

}

/*
Any point on segment AB can be written as:

text
P = A + t * (B - A)     for t in [0, 1]
t = 0 → you're at A

t = 1 → you're at B

t = 0.3 → you're 30% of the way from A to B

Same for segment CD:

text
P = C + u * (D - C)     for u in [0, 1]
The intersection point is the same P on both segments:

text
A + t * (B - A) = C + u * (D - C)
Let:

text
r = B - A      (direction of AB)
s = D - C      (direction of CD)
q = C - A      (vector from A to C)
Then the equation becomes:

text
A + t*r = C + u*s
Subtract A from both sides:

text
t*r = q + u*s
This is a vector equation. Two unknowns (t and u), two equations (x and y). Solve for t and u.

Solving for t and u using cross products
Take the cross product of both sides with s:

text
cross(t*r, s) = cross(q + u*s, s)
t * cross(r, s) = cross(q, s) + u * cross(s, s)
cross(s, s) = 0, so:

text
t = cross(q, s) / cross(r, s)
Similarly, take the cross product with r:

text
cross(t*r, r) = cross(q + u*s, r)
t * cross(r, r) = cross(q, r) + u * cross(s, r)
cross(r, r) = 0, and cross(s, r) = -cross(r, s), so:

text
u = cross(q, r) / cross(r, s)
So:

text
den = cross(r, s)
t   = cross(q, s) / den
u   = cross(q, r) / den
den is the same denominator in both. If den == 0, the segments are parallel and this formula breaks down (that's the collinear case you handle separately).

Getting the point from t
Once you have t:

text
x = A.x + t * r.x
y = A.y + t * r.y
Or equivalently using u on segment CD:

text
x = C.x + u * s.x
y = C.y + u * s.y
Both give the same point if your math is right. Use t (simpler).

You only need this formula in Case 1: proper crossing. In that case:

den != 0 (not parallel)

0 < t < 1 and 0 < u < 1 (the crossing is strictly inside both segments)

For Case 2 (endpoint touching), the intersection point is just the endpoint itself — no formula needed, just return c, d, a, or b.

For Case 3 (collinear overlap), there is no single intersection point, so you either return one point from the overlap or return a range. For a boolean-plus-single-point function, return one endpoint of the overlap.

*/




/*
    Point-to-segment distance:

    Given segment AB and a point P, find the shortest distance from P
    to any point on the segment AB.

    Let:
        r = B - A              // direction of the segment
        t = dot(P - A, r) / dot(r, r)

    The parameter t tells where the perpendicular foot of P lands on
    the INFINITE line through AB:
        t = 0   -> A
        t = 1   -> B
        t < 0   -> before A (on the extension)
        t > 1   -> after B  (on the extension)

    But the segment only covers 0 <= t <= 1. If the foot falls outside
    that range, the closest point on the SEGMENT is the nearest endpoint.

    Clamp t into [0, 1] to handle all three cases in one formula:

        tc    = clamp(t, 0, 1)
        Q     = A + tc * r      // closest point on the segment
        dist  = |P - Q|

    Why clamping works:
        - 0 <= t <= 1  -> clamp keeps t      -> Q is the perpendicular foot
        - t < 0        -> clamp gives 0      -> Q is A
        - t > 1        -> clamp gives 1      -> Q is B

    Degenerate case:
        If A == B (zero-length segment), r = (0,0) and dot(r,r) = 0,
        so the division is undefined. Handle it separately:
            dist = |P - A|
*/

double pointToSegmentDistance(Point p , Segment AB){
    Point r = AB.end - AB.start; //r is a direction of the AB segement! segment is just a finite line! (line is considered to be infinite)
    /*
        on chizi ke jalebe ine ke chon formule P=A+t*(B-A) ya hamoon P=A+t*r eqn segment hast vaghti jaye P mizari p (p pointe hast ke roye segment lozoman nist)
        va bad t = (p-A).r / r.r ro hal mikuni engari farz kardi p roye line hast (engari projection)
    */
   
    if(dotProduct2D(r,r)==0)
    {
        Point diff = p-AB.start;
        return std::sqrt(diff.x*diff.x + diff.y*diff.y);

    }

    double t = dotProduct2D(p-AB.start,r)/dotProduct2D(r,r);
    double t_clamp = std::clamp(t,0.0,1.0);
    Point closest_point = AB.start + r*t_clamp;

    Point diff = p-closest_point;
    return std::sqrt(diff.x*diff.x + diff.y*diff.y);
    

}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

/*
    ============================================================
    POLYGON BASICS
    ============================================================

    Representation:
        A polygon is an ordered list of vertices: vector<Point>.
        Consecutive vertices form edges:
            edge i: (V[i], V[(i+1) % n])
        The last edge wraps from V[n-1] back to V[0]. The % n is
        what "closes" the polygon. There is no separate Edge type;
        every edge is just two consecutive Points.

    Winding order:
        Vertices can be listed CCW or CW. Detect with the shoelace
        formula (signed double-area):

            2A = sum_i ( V[i].x * V[i+1].y - V[i+1].x * V[i].y )

        Sign of 2A:
            > 0  -> CCW
            < 0  -> CW
            = 0  -> degenerate (all points collinear)

        Why it works (the sweep intuition):
            Fix the origin O. As a point Q walks along the boundary,
            the segment OQ sweeps out signed area. Each term
            V[i] x V[i+1] is twice the signed area of triangle
            O V[i] V[i+1]. Summing over all edges:
                - regions swept twice with opposite orientation cancel
                - only the interior of the polygon survives
            So the sum = twice the signed area of the polygon.

    Point-in-polygon (ray casting / crossing number):
        Cast a ray from P to the right (y = P.y, x > P.x).
        Count how many polygon edges the ray crosses.
            odd  -> inside
            even -> outside

        For edge A->B, the edge crosses the ray iff:
            1) (A.y > P.y) != (B.y > P.y)          // spans y = P.y
            2) x_cross = A.x + (P.y - A.y) * (B.x - A.x) / (B.y - A.y)
            3) x_cross > P.x                        // crossing is to the right

        The asymmetric strict/non-strict comparison in (1) is what
        prevents double-counting when the ray passes exactly through
        a vertex: exactly one of the two adjacent edges satisfies it.

    Segment vs polygon intersection:
        Segment PQ intersects the polygon iff:
            - P is inside the polygon, OR
            - Q is inside the polygon, OR
            - PQ intersects some polygon edge.

        All three checks are required:
            - A segment entirely inside crosses no edge,
              so the endpoint tests catch it.
            - A segment entirely outside can still cross the boundary,
              so the edge tests catch it.

    Assumptions:
        - Polygon is simple (no self-intersections).
        - For point-in-polygon, behavior on the boundary is not
          guaranteed; treat it as a tie case.
*/


struct boundingBox{
    double minX;
    double minY;
    double maxX;
    double maxY;
};


enum class Offset{
    OUTWARD,
    INWARD,
};
struct Polygon;

Orientation polygonDirection(Polygon poly);
struct Polygon{
    Polygon(){}
    size_t size() {
        return points_.size();
    }
    boundingBox getBox(){
        if(points_.empty()) throw std::runtime_error("EMPTY POLYGON");
        boundingBox box;
        box.minX = offset_points_[0].x;
        box.maxX = offset_points_[0].x;
        box.minY = offset_points_[0].y;
        box.maxY = offset_points_[0].y;
        for (auto p : offset_points_){
            box.minX = std::min(box.minX , p.x);
            box.minY = std::min(box.minY , p.y);
            box.maxX = std::max(box.maxX , p.x);
            box.maxY = std::max(box.maxY , p.y);
        }
        return box;
    }

    std::pair<double,double> getProjectionRange(Point unit_direction){
        // first  of all be careful the direction must be unit (normalized)
        // imagine you want to find the min-max range of a polygon along an arbitrary direction --> useful when you wanna create agnles lanes so you have to find the min-max range of the polygon along the normal vector of the angled lanes!
        // so overall we use dot prodcut and assume each point is a vector from zero to that point to see how much of that vector is in the unit_direction!
        normalize(unit_direction);
        std::vector<double> how_far_along; 
        for (const auto& p : offset_points_){
            how_far_along.push_back(dotProduct2D(p,unit_direction));
        }
        auto a = *std::min_element(how_far_along.begin() , how_far_along.end());
        auto b = *std::max_element(how_far_along.begin() , how_far_along.end());
        return std::make_pair(a,b);
    }

    void normalize(Point& dir){
        double den = std::sqrt(dir.x*dir.x + dir.y*dir.y);
        dir.x = dir.x/den;
        dir.y = dir.y/den;
    }


    Orientation direction(){
        return polygonDirection(*this);
    }
   /**
    * Unit normal of 2D vector v = (x, y), via cross product with Z axis:
    *     ẑ × v = (0, 0, 1) × (x, y, 0) = (-y, x, 0)
    * Drop the always-zero z-component -> (-y, x), then normalize.
    */
    void calcInwardUnitNormalVector(){
        normal_vectors_.clear();
        Orientation dir = direction();
        if (dir == Orientation::CW){
            // here you need to cross product the segment with the z axis so the resulting normal vector would be:
            /*
                i j k
                x y 0
                0 0 1

                so the vector would be the determinent of the above matrix:
                y i - j x
            */
            for (int i = 0 ; i <points_.size() ; i++){
                Point A = points_.at(i);
                Point B = points_.at((i+1)%points_.size());
                double dx = (B-A).x;
                double dy = (B-A).y;
                normal_vectors_.push_back(Point{dy,-dx});
            }

        }
        else if(dir == Orientation::CCW){
            // here you need to cross product the z axis with the line so the resulting normal vector would be:
            /*
                i j k
                0 0 1
                x y 0

                so the vector would be the determinent of the above matrix:
                -y i + j x
            */
            for (int i = 0 ; i <points_.size() ; i++){
                Point A = points_.at(i);
                Point B = points_.at((i+1)%points_.size());
                double dx = (B-A).x;
                double dy = (B-A).y;
                normal_vectors_.push_back(Point{-dy,dx});
            }

        }
        else{
            std::cout<<"WHAT?\n";
        }


        for (auto& vec : normal_vectors_){
            double den = std::sqrt (vec.x*vec.x + vec.y*vec.y );
            vec.x = vec.x / den;
            vec.y = vec.y / den;
        }

    }


    // For each edge A -> B:
    // 1. Shift A and B inward by r along the edge's inward unit normal.
    //      A' = A + r * n
    //      B' = B + r * n
    // 2. The shifted points define an offset line.
    // 3. Intersect each pair of neighboring offset lines.
    // 4. Their intersection becomes the new vertex of the safe polygon.
    //    Line intersection:
    //      t = cross(C-A, s) / cross(r, s)
    //      P = A + t*r

    // Treat offset edges as infinite lines when constructing the new vertices.
    // We only need their intersection; t and u do not need to be in [0,1].
    // This avoids treating the offset operation as a finite-segment intersection.
    // TO UNDERSTAND IT YOU CAN PICTURE A SIMPLE CONCAVE POLYGON
    void calcOffsetPoints(double r, Offset  offset_){
        if (r <= 0 ){
            offset_points_ = points_;
            return;
        }
        offset_points_.clear();
        calcInwardUnitNormalVector();



        std::vector<Segment> offset_segs_;
        for (int i = 0 ; i <points_.size() ; i++){
            Point normal = normal_vectors_.at(i);
            if(offset_==Offset::OUTWARD){
                normal = normal * -1.0;
            }


            Point A_prime = points_.at(i) + normal*r;
            Point B_Prime = points_.at((i+1)%points_.size()) + normal*r;
            offset_segs_.push_back(Segment{A_prime, B_Prime});

        }
        for (int i = 0 ; i < offset_segs_.size() ; i++){
            // I wanna write a segment intersection again!
            Segment AB = offset_segs_.at(i);
            Segment CD = offset_segs_.at((i+1)%offset_segs_.size());
            Point A = AB.start;
            Point B = AB.end;
            Point C = CD.start;
            Point D = CD.end;

            Point r = B - A;
            Point q = C - A;
            Point s = D - C; 
            double den = crossProduct2D(r,s);
            if (den!=0){
                double t = crossProduct2D(q,s)/den;
                double u = crossProduct2D(q,r)/den;
                // Offset edges are treated as infinite lines.
                // Therefore t and u are unrestricted.
                Point intersection = A + r * t;
                offset_points_.push_back(intersection);
            }
            
        }

    }
 



    // Assuming the lanes are not touching the boudnary because if they do then the collinear case of a point touhcing the boundary also returns true for segemtn intersection function and ignores a perfectly valid transition
    bool isSegmentIntersectingWithPolygon(Segment a){
        for (int i = 0 ; i < offset_points_.size() ; i++){
            bool isIntersecting = segmantIntersection(Segment{offset_points_.at(i), offset_points_.at((i+1)%offset_points_.size())},a);
            if (isIntersecting)
                return true;
        }
        return false;
    }
    std::vector<Point> points_;
    std::vector<Point> offset_points_; // after inward minkowski by some circular robot footprint;
    std::vector<Point> normal_vectors_;
};

double signedPolygonArea(Polygon poly){
    double area = 0;
    for (size_t i = 0 ; i <poly.size() ; i++){
        area += (0.5)*crossProduct2D(poly.points_[i],poly.points_[(i+1)%poly.size()]); 
    }
    return area;
}
Orientation polygonDirection(Polygon poly){
    double signedArea = signedPolygonArea(poly);
    if(signedArea < 0 )
        return Orientation::CW;
    if (signedArea > 0 )
        return Orientation::CCW;
    return Orientation::COLLINEAR; // Or you could say degenrate case
}


/*
    Point-in-polygon using the Ray Casting / Even-Odd Rule:

    Goal:
        Determine whether point P is INSIDE or OUTSIDE the polygon.

    Idea:
        Shoot a horizontal ray from P toward +X:

            P ●--------------------->

        Count how many times this ray crosses the polygon boundary.

            odd  number of crossings -> INSIDE
            even number of crossings -> OUTSIDE

    For every polygon edge A -> B:

    1. Check whether the edge crosses the horizontal line y = P.y.

       P.y must lie between A.y and B.y:

           (A.y > P.y) != (B.y > P.y)

       If both endpoints are on the same side of P.y,
       the edge cannot cross P's horizontal line.

    2. If it crosses, find WHERE it crosses that horizontal line.

       Points on edge AB can be written parametrically as:

           X = A + t * (B - A)

       Therefore:

           X.y = A.y + t * (B.y - A.y)

       We want X.y = P.y, so solve for t:

           t = (P.y - A.y) / (B.y - A.y)

    3. Use t to calculate the x-coordinate of the intersection:

           X.x = A.x + t * (B.x - A.x)

       We only need X.x because our ray goes horizontally.

    4. Check whether the intersection is actually on the ray
       extending RIGHT from P:

           X.x > P.x

       If yes, we found one ray/polygon crossing.

    5. Every crossing toggles the state:

           inside = !inside;

       Start with:

           inside = false;

       After checking all edges:

           inside == true  -> INSIDE
           inside == false -> OUTSIDE

    Important:
        Handle the case where P lies exactly on a polygon edge
        separately (BOUNDARY), before/alongside ray casting.
*/


enum class PointState{
    INSIDE,
    OUTSIDE,
    BOUNDARY
};


PointState pointInPolygon(Point p , Polygon poly){
    
    int count = 0;    
    for (size_t i = 0 ; i <poly.size() ; i++){
        Point A = poly.points_[i];
        Point B = poly.points_[(i+1)%poly.size()];
        Orientation test = orientationTest(A,B,p);
        if(test==Orientation::COLLINEAR){
            // bounding box
            if (p.x >= std::min(A.x,B.x)&&
                p.x <= std::max(A.x,B.x)&&
                p.y >= std::min(A.y,B.y)&&
                p.y <= std::max(A.y,B.y))
            {
                return PointState::BOUNDARY;
            }
        }


        //Is one endpoint above P and the other below P?
        if ((A.y > p.y) != (B.y > p.y)) {

            // now lets see the intersection point to see if if on the left or right!
            // we assume the ray is horizontal line to the right of p

            Point r = B - A;

            double t = (p.y - A.y) / r.y;

            double xIntersection = A.x + t * r.x;
            if(xIntersection > p.x){
                count++;
            }
        }
    }



    if (count%2==1){
        std::cout<<"INSIDE\n";
        return PointState::INSIDE;
        
    }
    else{
        std::cout<<"OUTSIDE\n";
        return PointState::OUTSIDE;
    }


}


/*


    Given a horizontal sweep line y = constant, find the portions of the
    line that lie inside the polygon.

    1. Find where the sweep line intersects every polygon edge.
    2. Sort the intersection x-coordinates from left to right.
    3. Along the sweep line, we alternate between OUTSIDE and INSIDE
       each time we cross a polygon boundary.
    4. Therefore, consecutive pairs of intersections form inside segments:
       
           x0 ---- x1    x2 ---- x3
            INSIDE        INSIDE

       So we create segments [x0,x1], [x2,x3], etc.

    Example:
        intersections = [1, 4, 6, 10]
        coverage segments = [1,4] and [6,10]

*/

std::vector<Segment> getCoverageSegments(Polygon poly, double y){
    std::vector<double> x_intersections_;
    for (size_t i = 0 ; i<poly.size() ; i++){
        Point A = poly.offset_points_[i];
        Point B = poly.offset_points_[(i+1)%poly.size()];
        // Since the line is horiziontal the check is easier for intersection

        // bool isYaboveA = y > A.y;
        // bool isYaboveB = y > B.y;
        // if (isYaboveA != isYaboveB){

        bool isYaboveOrAtA = y >= A.y;
        bool isYaboveOrAtB = y >= B.y;
        bool isYbelowA = y < A.y;
        bool isYbelowB = y <B.y;
        if ((isYaboveOrAtA && isYbelowB) || (isYaboveOrAtB && isYbelowA) ){
            // So Y is above exactly one of them so A and B are on the opposite side of the y so there is intersection!
            Point r = B-A;
            double t = (y - A.y)/r.y;
            double xIntersection;
            xIntersection = A.x + t*r.x; 
            x_intersections_.push_back(xIntersection);
            std::cout<<"t:"<<t<<", x:"<<xIntersection<<"\n";
        }
        

    }
    std::sort(x_intersections_.begin() , x_intersections_.end());
    std::vector<Segment> regions;
    for (int i = 0 ; i <x_intersections_.size() ; i+=2){
        Point A{x_intersections_.at(i) , y};
        Point B{x_intersections_.at(i+1) , y};
        regions.push_back(Segment(A,B));
    }
    return regions;

}

/*
// arbitrary line (not necessarily horizontal or vertical)
// Line AB is infinite, while polygon edge CD is finite.

// No orientation test is needed here:
// We directly solve the parametric intersection:
//     A + t*r = C + u*s
// Since AB is infinite, t is unrestricted.
// Since CD is finite, only 0 <= u <= 1 is required.

// t gives the position of the intersection along the infinite line.
// u tells whether the intersection lies on the finite polygon edge.


// Sorting options:
// 1. Store t together with each intersection and sort by t directly.
// 2. Recalculate t from each Point using the dot product:
//      t = dot(X-A, r) / dot(r,r)
// 3. Use a std::sort lambda and calculate t inside the comparator.
// Here we use option 3, since we only store Points.

// Note: the t calculated above using cross products and the t
// calculated using the dot product are mathematically identical.

*/
// MIND THAT AB is infinite so its a line not a segment!
std::vector<Segment> getCoverageSegments(Polygon poly, Point A , Point B)
{
    std::vector<Point> points_;
    Point r = B - A;
    for (int i = 0 ; i < poly.size() ; i++){
        Point C = poly.offset_points_[i];
        Point D = poly.offset_points_[(i+1)%poly.size()];

        Point s = D - C;
        Point q = C - A; 
        
        double den = crossProduct2D(r,s);
        if (den!=0){
            double t = crossProduct2D(q,s)/den;
            std::cout<<"t:"<<t<<"\n";
            double u = crossProduct2D(q,r)/den;
            if (u>=0 && u<=1){
                double xIntersection = C.x + u*s.x;
                double yIntersection = C.y + u*s.y;
                double t1 = dotProduct2D(Point{xIntersection,yIntersection} - A , r)/dotProduct2D(r,r); // REDUNDANT! --> gives the same t that we calced using cross prodcut!
                std::cout<<"t1: "<<t1<<"\n";
                points_.push_back(Point{xIntersection,yIntersection});
            }
        } 
    }
    std::sort(points_.begin(), points_.end() , [&](Point a, Point b){
        double t1 = dotProduct2D(a-A,r) / dotProduct2D(r,r);
        double t2 = dotProduct2D(b-A,r) / dotProduct2D(r,r);
        return t1<t2;
    });

    std::vector<Segment> segs_;
    for (int i = 0 ; i < points_.size() ; i += 2){
        segs_.push_back(Segment{Point{points_.at(i)}, Point{points_.at(i+1)}});
    }
    return segs_;

}


struct Trajectory{
    std::vector<Segment> segs_;
};

// Lanes are between the spacings!
Trajectory createLanesInPolygon(Polygon poly , double spacing, double footprint = 0){
    
    poly.calcOffsetPoints(footprint, Offset::INWARD);

    // Horizontal first! from y = minY to maxY!
    boundingBox box = poly.getBox();
    int count = (box.maxY - box.minY) / spacing;
    Trajectory traj_;
    Point previous_point_{0,0};
    int lane_count = 0;
    // starting the direction from left to right!
    for (int i = 0 ; i < count ; i++)
    {
        
        std::vector<Segment> current_segs_ = getCoverageSegments(poly, box.minY + (i+0.5)*spacing);
        if (current_segs_.empty())
            continue;
        lane_count++;

        for (auto& segs_ : current_segs_) segs_.type_ = Type::COVERAGE;


        if (lane_count%2 == 1) 
        {

            if (lane_count>1) {
                // TRANSITION BETWEEN LANES?
                Point current_point_{0,0};
                current_point_ = current_segs_.front().start;

                Segment transition_seg_{previous_point_,current_point_};
                // if(poly.isSegmentIntersectingWithPolygon(transition_seg_)){
                //     //someother logic --> LIKE USING A*
                // }
                // else{
                    transition_seg_.type_ = Type::TRANSITION;
                    traj_.segs_.push_back(transition_seg_);

                // }

            }
            traj_.segs_.insert(traj_.segs_.end() , current_segs_.begin() , current_segs_.end());
            previous_point_ = current_segs_.back().end;
        }
        else{
            for (auto& segs_ : current_segs_) segs_.reverseSeg(); 
            std::reverse(current_segs_.begin() , current_segs_.end());
            if (lane_count>1) {
                // TRANSITION BETWEEN LANES?
                Point current_point_{0,0};
                current_point_ = current_segs_.front().start;

                Segment transition_seg_{previous_point_,current_point_};
                // if (poly.isSegmentIntersectingWithPolygon(transition_seg_)){
                //     //someother logic --> LIKE USING A*!
                // }
                // else{
                    transition_seg_.type_ = Type::TRANSITION;
                    traj_.segs_.push_back(transition_seg_);

                // }

            }
            traj_.segs_.insert(traj_.segs_.end() , current_segs_.begin() , current_segs_.end()); 
            previous_point_ = current_segs_.back().end;
        }



    }
    return traj_;

}

// double piToPi(double theta){
//     if (theta > 0 ){
//         while (theta > M_PI)
//             theta -= 2*M_PI;
//     }
//     if (theta < 0 ){
//         while (theta <=-M_PI)
//             theta+= 2*M_PI;
//     }
//     return theta;
// }
void piToPi(double& theta){
    theta = fmod(theta, 2*M_PI);
    if (theta > M_PI) theta-= 2*M_PI;
    if (theta <= -M_PI) theta+= 2*M_PI;
}


// it gives different normal if you cross product to z or from z! i use from z i guess it wouldnt matter for the min-max projection reason im using this function for!
Point getUnitNormalToVector(Point p){
    double den = std::sqrt(p.x*p.x+p.y*p.y);
    return Point{-p.y/den , p.x/den};
}

// Create Angled Lanes in Polygon! so theta is the lane direction! so in order to figure out the min-max range you need to use the normal vector of that theta direction
Trajectory createAngledLanesInPolygon(Polygon poly , double spacing, double theta ,double footprint = 0){
    poly.calcOffsetPoints(footprint, Offset::INWARD);
    piToPi(theta); 
    Point lane_dir{cos(theta),sin(theta)};
    Point normal = getUnitNormalToVector(lane_dir);
    auto [vmin,vmax] = poly.getProjectionRange(normal);
    std::cout<<"VMIN:" <<vmin <<" , VMAX:"<<vmax<<"\n";

    Trajectory traj_;
    int count = (vmax - vmin) / spacing;
    int lane_number_ = 0;
    Point prev_point_{0,0};
    for ( int i = 0 ; i < count ; i ++){
        Point A = normal*(vmin+(i+0.5)*spacing); 
        double arbitrary_multiplier = 1 ; // dont use zero
        Point B = A + lane_dir * arbitrary_multiplier;
        auto segs_ = getCoverageSegments(poly,A,B);
        for(auto& s : segs_){s.type_=Type::COVERAGE;}
        if(segs_.empty()) continue;
        lane_number_++; 
        if (lane_number_%2 == 1) // it covers the left to right lanes! (starting with this)
        {
            if(lane_number_> 1){
                Point A = prev_point_;
                Point B = segs_.front().start; 
                Segment s_{A,B};
                s_.type_ = Type::TRANSITION;
                traj_.segs_.push_back(s_);
            }
            traj_.segs_.insert(traj_.segs_.end() , segs_.begin() , segs_.end());
            prev_point_= segs_.back().end;
        }
        else{
            std::reverse(segs_.begin() , segs_.end());
            for (auto& s : segs_) s.reverseSeg();

            if(lane_number_>1){
                Point A = prev_point_;
                Point B = segs_.front().start; 
                Segment s_{A,B};
                s_.type_ = Type::TRANSITION;
                traj_.segs_.push_back(s_);
            }

            traj_.segs_.insert(traj_.segs_.end(),segs_.begin(),segs_.end());
            prev_point_= segs_.back().end;
        }

    }

    return traj_;

}


int main(int argc , char* argv[]){
    OrientationLog.insert(std::pair<int, std::string>(0,"CW"));
    OrientationLog.insert(std::pair<int, std::string>(1,"CCW"));
    OrientationLog.insert(std::pair<int, std::string>(2,"COLINEAR"));



    Point a(2,3);
    Point b(1,1);
    Point c = a - b;
    c.print();
    Point d = a + b;
    d.print();

    double r = crossProduct2D(Point{2,0}, Point(1,1));
    std::cout<<r<<"\n";
    Orientation o = orientationTest(Point(0,0), Point(2,0), Point(1,1)); // between two vectors!

    std::cout<< OrientationLog[static_cast<int>(o)]<<"\n";
    // Segment vec_from_a_to_b =  ;    

    /////////////////////
    Point p1(0,0);
    Point p2(3,3);
    Point p3(1,1);
    Point p4(2.5,2.5);
    Segment s1(p1,p2);
    Segment s2(p3,p4);
    std::cout<<"ANY INTERSECTION?"<<segmantIntersection(s1,s2)<<"\n";
    //////////////////////
    Segment s3(Point{0,0},Point{1,1});
    Point p5{0.0,1.0};
    std::cout<<"point to seg dis: "<<pointToSegmentDistance(p5,s3)<<"\n";


    ////////////////POLYGON//////////////
    std::cout<<"Polygon \n";
    Polygon poly_;
    poly_.points_.push_back(Point{1,1});
    poly_.points_.push_back(Point{5,1});
    poly_.points_.push_back(Point{4,2});
    poly_.points_.push_back(Point{2,2});

    std::cout<<"AREA: "<<signedPolygonArea(poly_)<<"\n";
    Orientation dir = polygonDirection(poly_);
    std::cout<<OrientationLog.at(static_cast<int>(dir))<<"\n";
    int pip = static_cast<int>(pointInPolygon(Point{1.5,1.5},poly_));
    if(pip==0)
        std::cout<<"Point in Polygon?: yes"<<"\n";
    else if (pip==1)
        std::cout<<"Point in Polygon?: no"<<"\n";
    else
        std::cout<<"Point in Polygon?: boundary"<<"\n";

    /////////////////////////////////////
    // //////////////// COVERAGE SEGMENTS ////////////////

    // Polygon coverage_poly;

    // // U-shaped concave polygon
    // //
    // //       (4,5)---(6,5)
    // //        |         |
    // //        |         |
    // //       (4,3)     (6,3)
    // //        |         |
    // // (1,1)--------------(9,1)
    // //   |                    |
    // //   +--------------------+
    // //
    // // At y = 4:
    // //
    // // x = 1 ---- x = 4       x = 6 ---- x = 9
    // //       INSIDE                 INSIDE

    // coverage_poly.points_.push_back(Point{1,1});
    // coverage_poly.points_.push_back(Point{9,1});
    // coverage_poly.points_.push_back(Point{9,5});
    // coverage_poly.points_.push_back(Point{6,5});
    // coverage_poly.points_.push_back(Point{6,3});
    // coverage_poly.points_.push_back(Point{4,3});
    // coverage_poly.points_.push_back(Point{4,5});
    // coverage_poly.points_.push_back(Point{1,5});

    // double sweep_y = 3.0;

    // std::vector<Segment> coverage_segments =
    //     getCoverageSegments(coverage_poly, sweep_y);

    // std::cout << "Coverage segments:\n";

    // for (const auto& segment : coverage_segments) {
    //     std::cout << "("
    //             << segment.start.x << ", " << segment.start.y
    //             << ") -> ("
    //             << segment.end.x << ", " << segment.end.y
    //             << ")\n";
    // }
    // ////////////////////////////
    // std::cout<<"-----------------\n";
    // getCoverageSegments(coverage_poly, Point{0,2}, Point{10,2});
    //////////////// CONCAVE COVERAGE TEST //////////////////

    Polygon concave_poly;

    // L-shaped polygon:
    //
    // (1,1) -------- (7,1)
    //   |               |
    //   |               |
    //   |       (5,4) --+
    //   |        |
    //   |        |
    //   |        |
    // (1,7) ----- (5,7)
    //
    // Actually represented as:
    //       (1,7) -------- (5,7)
    //         |              |
    //         |              |
    //         |              (5,4)
    //         |                |
    //         |                |
    //         |                |
    //         |                |
    //         +----------------(7,4)
    //         |
    //         |
    //       (1,1) ------------(7,1)

    concave_poly.points_.push_back(Point{1,1});
    concave_poly.points_.push_back(Point{7,1});
    concave_poly.points_.push_back(Point{7,4});
    concave_poly.points_.push_back(Point{5,4});
    concave_poly.points_.push_back(Point{5,7});
    concave_poly.points_.push_back(Point{1,7});

    // Robot footprint
    double spacing = 1.0;
    double footprint = 0.5;
    Trajectory trajectory = createLanesInPolygon(concave_poly, spacing,footprint);

    std::cout << "CONCAVE TRAJECTORY\n";

    for (const auto& seg : trajectory.segs_) {

        std::cout
            << (seg.type_ == Type::COVERAGE ? "COVERAGE   " : "TRANSITION  ")
            << "(" << seg.start.x << ", " << seg.start.y << ")"
            << " -> "
            << "(" << seg.end.x << ", " << seg.end.y << ")"
            << "\n";
    }

    


    double angle_in_radians = 1.0;

    Trajectory trajectory2 = createAngledLanesInPolygon(concave_poly, spacing, angle_in_radians,footprint);

    std::cout << "CONCAVE TRAJECTORY USING ANGLE\n";

    for (const auto& seg : trajectory2.segs_) {

        std::cout
            << (seg.type_ == Type::COVERAGE ? "COVERAGE   " : "TRANSITION  ")
            << "(" << seg.start.x << ", " << seg.start.y << ")"
            << " -> "
            << "(" << seg.end.x << ", " << seg.end.y << ")"
            << "\n";
    }



}


