#include "cleaner/rasterized_image.hpp"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>

RasterizedImage::RasterizedImage(std::string path_, double resolution, Point<double> grid_origin_in_world): resolution_(resolution), grid_origin_in_world_(grid_origin_in_world){

    size_t last_slash = std::string::npos;
    for (int k = 0; k<path_.size() ; k++){
       if(path_[k]=='/') {
        last_slash = k;
       }
    }
    size_t start = (last_slash == std::string::npos) ? 0 : last_slash + 1;
    for(size_t k = start ; k < path_.size() ; ++k){
        image_name_ += path_[k];
    }


    image_ = cv::imread(path_, cv::IMREAD_GRAYSCALE);
    if(image_.empty()){
        throw std::runtime_error("Failed to load image " + path_);
    }
    height_ = image_.size().height;
    width_ = image_.size().width;

    if(resolution_<=0.0){
        throw std::invalid_argument("Resolution must be greater than zero");
    }

}

void RasterizedImage::showImage() const {
    if(image_.empty()){
        return;
    }
    cv::imshow(image_name_, image_);
    cv::waitKey(0);
    cv::destroyAllWindows();
}
const cv::Mat& RasterizedImage::getImage() const{
    return image_;
}

/*
 * Coordinate transform notation:
 *
 *      ᴬp = ᴬRᴮ · ᴮp + ᴬtᴮ
 *
 * where:
 *
 *      ᴬp   = point p expressed in frame A
 *      ᴮp   = point p expressed in frame B
 *
 *      ᴬRᴮ  = rotation from frame B coordinates into frame A coordinates
 *
 *      ᴬtᴮ  = position of the origin of frame B,
 *              expressed in frame A
 *
 *
 * Therefore:
 *
 *      ᴬp = ᴬTᴮ · ᴮp
 *
 * with
 *
 *              [ ᴬRᴮ   ᴬtᴮ ]
 *      ᴬTᴮ  =  [             ]
 *              [  0      1   ]
 *
 *
 * Opposite direction:
 *
 *      ᴮp = ᴮRᴬ · ᴬp + ᴮtᴬ
 *
 * or
 *
 *      ᴮp = ᴮTᴬ · ᴬp
 *
 *
 * Inverse relations:
 *
 *      ᴮRᴬ = (ᴬRᴮ)ᵀ
 *
 *      ᴮtᴬ = -(ᴬRᴮ)ᵀ · ᴬtᴮ
 *
 *      ᴮTᴬ = (ᴬTᴮ)⁻¹
 *
 *
 * Memory rule:
 *
 *      ᴬTᴮ = pose of frame B as seen from frame A
 *
 * The LEFT superscript tells you:
 *      "which frame the result is expressed in"
 *
 * The RIGHT superscript tells you:
 *      "which frame/object is being described"
 *
 *
 * Sanity check:
 *
 *      if ᴮp = [0, 0]ᵀ
 *
 * then
 *
 *      ᴬp = ᴬtᴮ
 *
 * so ᴬtᴮ is exactly the location of the B-origin,
 * expressed in frame A.
 */
// Gives the position of the center of the cell wrt World frame considering the resolution
Point<double> RasterizedImage::cellCenterToWorld(int row, int col) const {
    if (!inBounds(row, col)) throw std::out_of_range("Cell index out of bounds");


    // there are two trasnformation involved! one is from top left to bottom left keeping the y direction downward and rotation matrix has zero theta!
    /*
        ᴬp = ᴬRᴮ · ᴮp + ᴬtᴮ   --> mind that ᴬtᴮ means ᴬt_B  mainly whats orign of B in the A coord frame  
    */

    // OpenCV gives us the discrete cell index (row, col), not a continuous geometric point.
    // We represent index (row, col) as the TOP-LEFT corner of a square cell that extends
    // resolution_ units to the right and resolution_ units downward in the image frame.
    // Therefore the geometric center of that cell is half a cell away from that corner,
    // so we add resolution_/2 to both x and y.
    double Px_IM = col*resolution_ + resolution_/2; // 
    double Py_IM = row*resolution_ + resolution_/2;


    Point<double> P_IM{Px_IM,Py_IM};
    Point<double> t_im_W_prime{0.0,-height_*resolution_}; // origin of Image coord in W' frame
    // assuming rotation matrix is identity because theta is zero between IM and W_PRIME
    Point<double> P_W_prime{Px_IM, Py_IM - height_*resolution_};

    // And the real world coordinate is on the same place as W_PRIME but has its y upward! --> so negate the y coord
    // ᵂtᴳ: position of the bottom-left OUTER boundary of the raster,
    // expressed in the world frame.
    Point<double> P_W{grid_origin_in_world_.x_ + P_W_prime.x_ , grid_origin_in_world_.y_-P_W_prime.y_};
    return P_W;

}

double RasterizedImage::getResolution() const {
    return resolution_;
}
uchar RasterizedImage::getIntensity(int row, int col) const {
    if (!inBounds(row, col)) throw std::out_of_range("Cell index out of bounds");
    return image_.at<uchar>(row,col);
}


/*
    White (255) : freespace
    Black (0) : obstacle
*/

void RasterizedImage::colorPixel(int row, int col) const {
    if (!inBounds(row, col)) throw std::out_of_range("Cell index out of bounds");
    cv::Mat debug_image;
    cv::cvtColor(image_, debug_image, cv::COLOR_GRAY2BGR);
    debug_image.at<cv::Vec3b>(row,col) = cv::Vec3b(0,0,255); //BGR convention not RGB
    cv::imshow("debug", debug_image);
    cv::waitKey(0);

}

int RasterizedImage::getHeight() const {
    return height_;
}
int RasterizedImage::getWidth() const {
    return width_;
}

CellState RasterizedImage::getCellState(int row, int col) const {
    const auto intensity = getIntensity(row, col);
    if (intensity > 128)
        return CellState::Free;
    return CellState::Occupied;
}