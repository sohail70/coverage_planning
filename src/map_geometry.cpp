#include "cleaner/map_geometry.hpp"
#include "cleaner/occupancy_grid.hpp"
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

MapGeometry::MapGeometry(const OccupancyGrid& grid_map): grid_map_(grid_map) {
    Contour image_frame_;
    image_frame_.id_ = 1;
    image_frame_.is_hole_ = false;
    image_frame_.parent_id_ = -1;
    // image_frame_.points_ --> later put the borders of the image frame in this!
    contours_.push_back(image_frame_);



    /*
        -1  = unknown
        0  = free/background
        1  = unprocessed obstacle
        2  = border #2
        3  = border #3
        4  = border #4
    */
    working_grid_.resize(grid_map_.getHeight(), std::vector<int>(grid_map_.getWidth(),0));
    for (int row = 0; row < grid_map_.getHeight(); ++row) {
        for (int col = 0; col < grid_map_.getWidth(); ++col) {
            switch (grid_map_.getCellState(row, col)) {
                case CellState::Free:     working_grid_[row][col] =  0; break;
                case CellState::Occupied: working_grid_[row][col] =  1; break;
                case CellState::Unknown:  working_grid_[row][col] = -1; break;
            }
        }
    }
    std::cout<<working_grid_.size()<<"\n";


    


}

void MapGeometry::findContours(){
    int contour_count_ = 1; // assuming frame of the picture is the first countor    
    int LNBD = 0; // the contours vector indices starts with 0 which is the frame index --> this variable is for tracking the border to assign parents in a hierarchy
    for(int row = 0 ; row < working_grid_.size() ; row++)
    {
        LNBD = 0; // so we reset to zero at each row because the assumption is the FRAME is the parent of all the obstalces inside the image!
        for(int col = 0 ; col < working_grid_.at(row).size() ; col++ ){
            if (working_grid_[row][col] > 1) LNBD = working_grid_[row][col]; // working_grid_ changes in the trace function 

            int current_cell_state_,left_cell_state_,right_cell_state_;
            current_cell_state_ = working_grid_[row][col];
            if(col > 0)
                left_cell_state_ = working_grid_[row][col-1];
            if(col < working_grid_.at(row).size() - 1)
                right_cell_state_ = working_grid_[row][col+1];


            // This is outer border cell scenario (already we are on obstacle and the left cell is free)
            if (current_cell_state_ == 1 && 
                left_cell_state_ == 0) {
                contour_count_++;
                Contour c;
                c.id_ = contour_count_;
                c.is_hole_ = false;
                c.parent_id_ = LNBD;
                c.points_.push_back(Point<int>{col,row}) ;
                working_grid_[row][col] = c.id_;
                trace(c,row, col, row, col-1, Point<int>{col,row});
                contours_.push_back(c);
            }
            // This is the hole scenraio (we are already on obstalce and the right side is free)
            // If we are on obstalce and both left and right are free then the first if above has the priority
            else if(current_cell_state_ == 1 &&
                right_cell_state_ == 0) {
                contour_count_++;
                Contour c;
                c.id_ = contour_count_;
                c.is_hole_ = true;
                c.parent_id_ = LNBD;
                c.points_.push_back(Point<int>{col,row}) ;
                working_grid_[row][col] = c.id_;
                trace(c,row, col, row, col+1, Point<int>{col,row});
                contours_.push_back(c);
            }

        }
    }
}



/*
     feel like i should use another struct for indinces that uses int instead of point which is in double or make point template!
*/
void MapGeometry::trace(Contour& c, int row, int col , int prev_row, int prev_col, const Point<int>& start_pixel){
    Point<int> current_index_{col,row};
    Point<int> previous_index{prev_col,prev_row};
    Point<int> dir = previous_index - current_index_;
    auto start = std::find(clockwise_.begin(),clockwise_.end(),dir);
    int start_index_ = static_cast<int>(start - clockwise_.begin()) ;

    bool found_neighbor = false;


    for(int i = start_index_ ; i <start_index_ + 8 ; i++){
        int current_neighbor_index_ = (i+1)%8;
        Point<int> step = clockwise_.at(current_neighbor_index_);
        Point<int> neighbor_index_ {col + step.x_ , row + step.y_}; 
        
        if (neighbor_index_.y_ >= grid_map_.getHeight() ||
            neighbor_index_.y_ < 0 ||
            neighbor_index_.x_ >= grid_map_.getWidth() ||
            neighbor_index_.x_ < 0)
            continue;

        int neighbor_state_ = working_grid_[neighbor_index_.y_][neighbor_index_.x_];

        if (neighbor_index_ == start_pixel){
            std::cout<<"FINISHED TRACING \n";
            return;
        } 
        if(neighbor_state_ == 1){
            found_neighbor = true;
            working_grid_[neighbor_index_.y_][neighbor_index_.x_] = c.id_;
            c.points_.push_back({neighbor_index_});
            previous_index = current_index_;
            current_index_ = neighbor_index_;        
            break;
        }
    }
    if(!found_neighbor){
        std::cout<< "No Next border pixel found \n";
        return;
    }
    trace(c,current_index_.y_,current_index_.x_, previous_index.y_ , previous_index.x_, start_pixel);
}


void MapGeometry::showContours(const cv::Mat& original_image)
{
    cv::Mat colored_image;
    cv::cvtColor(original_image, colored_image, cv::COLOR_GRAY2BGR);

    for (const auto& c : contours_) {

        for (const auto& p : c.points_) {

            colored_image.at<cv::Vec3b>(p.y_, p.x_) =
                cv::Vec3b(0, 0, 255);  // red
        }
    }

    cv::imshow("Contours", colored_image);
    cv::waitKey(0);
}