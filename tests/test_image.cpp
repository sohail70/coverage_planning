#include<iostream>
#include<cleaner/rasterized_image.hpp>


int main(int argc , char* argv[])
{
    RasterizedImage image_ ("../maps/map2.png",0.05 , Point<double>{0.0,0.0});

    image_.showImage();
    std::cout<<static_cast<int>(image_.getCellState(20, 40))<<"\n";
    image_.colorPixel(20, 40);
    Point p = image_.cellCenterToWorld(0, image_.getWidth()-1);
    std::cout<<p.x_<<" , "<<p.y_<<"\n";
    return 0;
}