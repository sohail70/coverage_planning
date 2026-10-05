#include "cleaner/map_geometry.hpp"
#include "cleaner/rasterized_image.hpp"
int main(int argc , char* argv[]){
    RasterizedImage image_ ("../maps/map2.png",0.05 , Point<double>{0.0,0.0});
    MapGeometry map_geometry_(image_) ;
    map_geometry_.findContours();
    map_geometry_.showContours(image_.getImage());
    map_geometry_.listContours();
    map_geometry_.polygonize();
}