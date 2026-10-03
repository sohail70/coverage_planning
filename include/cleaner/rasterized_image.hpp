#pragma once
#include "cleaner/occupancy_grid.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/mat.hpp>

/*
    I want this class to load the image from a file using OpenCV and just provide me the boundary of the obstalces region etc in the pixel coord frames!
    threshold the rasterized image from 0-255 to 0-1 to free and occupied space!
    stores resolution, origin
    and provide the pixel to worl coordinate conversion!
*/
class RasterizedImage : public OccupancyGrid{
    public:
        RasterizedImage(std::string path_, double resolution, Point<double> grid_origin_in_world);
        void showImage() const;
        const cv::Mat& getImage() const;
        /*
            image coordinate (opencv) is on top left and rows increase downward and cols
            increase to the right but in the world coordinate y increases upward so take care of this here
        */
        int getWidth() const override;
        int getHeight() const override;
        double getResolution() const override;
        uchar getIntensity(int row, int col) const;
        CellState getCellState(int row, int col) const override;
        Point<double> cellCenterToWorld(int row, int col) const override; 
        void colorPixel(int row,int col) const;

    private:
        cv::Mat image_;
        std::string image_name_;
        int height_;
        int width_;
        double resolution_; // how many meters is a pixel?
        Point<double> grid_origin_in_world_; //defined by the user!  World coordinate!  not image coordinate!



};