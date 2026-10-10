#include "cleaner/map_geometry.hpp"
#include "cleaner/rasterized_image.hpp"
int main(int argc , char* argv[]){
    RasterizedImage image_ ("../maps/map2.png",0.05 , Point<double>{0.0,0.0});
    MapGeometry map_geometry_(image_) ;
    map_geometry_.findContours();
    // map_geometry_.showContours(image_.getImage());
    // map_geometry_.showNBD(0 ,0, 400, 400);
    // map_geometry_.listContours();
    map_geometry_.polygonize();
    // map_geometry_.showCrackBoundaries(image_.getImage());
    map_geometry_.showSimplifiedBoundaries(image_.getImage());
    map_geometry_.determineReachableRegion(Point<double>{152,313});
    auto free_region_ = map_geometry_.getFreeRegions().at(0);
    map_geometry_.BCD(free_region_.geometry_);
    // BCD has now filled vertical_slices_.
    map_geometry_.showVerticalSlices(
        image_.getImage()
    );

}


/*
Exactly. Your next step is:
For each consecutive pair of those x-values, choose an x_sample strictly between them and determine all free y-intervals along that vertical line.

For example, your first slabs are:
(1, 74)
(74, 77)
(77, 153)
(153, 191)
...

For (74,77) you could choose:
x_sample = 75.5

Then imagine the vertical line:
x = 75.5

and ask: where does this line cross polygon boundaries?
Concrete procedure
For that x_sample:
1. Intersect the vertical line with every edge of:
   - the outer polygon
   - every hole polygon
2. Every intersection gives you a y.
Suppose you get:
y intersections:

1
36
202
978

Sort them:
1, 36, 202, 978

These divide the vertical line into candidate intervals:
(1,36)
(36,202)
(202,978)

Now test the midpoint of each candidate interval.
For example:
(x_sample, 18.5)
(x_sample, 119)
(x_sample, 590)

using the geometry you already wrote:
inside outer
AND
outside every hole

Then perhaps you discover:
(1,36)      FREE
(36,202)    obstacle
(202,978)   FREE

Therefore this slab has two free vertical intervals:
[1,36]

[202,978]

That's the data BCD wants.
Why use the midpoint test?
Because you already have pointInPolygon().
You don't have to invent complicated logic like:
first intersection means entering free space
second means obstacle
third means ...

Just ask directly:
Is a point between these two boundary crossings in the free-space region?

Much harder to get wrong.
How do you get the y intersections?
For every polygon edge:
A -------- B

ask whether the vertical line x = x_sample passes through that edge.
For a general edge:
A
 \
  \
   B

if x_sample lies between A.x and B.x, then compute the corresponding y.
Because your x_sample is deliberately between unique vertex x-values, it will never pass directly through a polygon vertex. This removes a lot of nasty intersection ambiguity.
For your current orthogonal raster polygons, it's even easier:
horizontal edge
----------------
        |
        | x_sample

gives one y intersection.
Vertical edges normally won't intersect your sample line because their x-coordinate is one of the critical event x-values, while x_sample lies strictly between two event x-values.
So your next data structure can conceptually be something as simple as:
struct VerticalInterval
{
    double y_min_;
    double y_max_;
};

and for each slab:
Slab (74,77):

    x_sample = 75.5

    free intervals:
        [1,36]
        [202,978]

Then after you can generate this for every slab, that is when we compare neighboring slabs to determine continuation, split, and merge.
So don't create BCD cells yet. Your next milestone is simply:
for each x-slab
    successfully print all free y-intervals

Once that output looks correct visually, the actual BCD connectivity step becomes much easier.


*/