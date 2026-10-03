#include "cleaner/map_geometry.hpp"
#include "cleaner/occupancy_grid.hpp"
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

MapGeometry::MapGeometry(const OccupancyGrid& grid_map): grid_map_(grid_map) {
    Contour image_frame_;
    image_frame_.id_ = 1;
    image_frame_.is_hole_ = false;
    image_frame_.parent_id_ = -1;
    // image_frame_.points_ --> later put the borders of the image frame in this!
    contours_.push_back(image_frame_);



    /*
        1 = traversable free space
        0 = non-traversable

        2  = border #2
        3  = border #3
        4  = border #4
    */
    const int H = grid_map_.getHeight();
    const int W = grid_map_.getWidth();
    /*
        PADDING --> one to the left one to the right so H+2 ,same for W
        So actuall map starts at row+1 and col+1

        000000000000000
        0             0
        0 actual map  0
        0             0
        000000000000000
    */

    working_grid_.resize(H+2 , std::vector<int>(W+2,0));
    for (int row = 0; row < grid_map_.getHeight(); ++row) {
        for (int col = 0; col < grid_map_.getWidth(); ++col) {
            switch (grid_map_.getCellState(row, col)) {
                case CellState::Free:
                    working_grid_[row+1][col+1] = 1;
                    break;

                case CellState::Occupied:
                case CellState::Unknown:
                    working_grid_[row + 1][col + 1] = 0;
                    break;
            }
        }
    }
    std::cout<<working_grid_.size()<<"\n";


    


}

void MapGeometry::findContours(){
    int contour_count_ = 1; // assuming frame of the picture is the first countor    
    int LNBD = 1; // the contours vector indices starts with 0 which is the frame index --> this variable is for tracking the border to assign parents in a hierarchy
    for(int row = 1 ; row < working_grid_.size() - 1 ; row++)
    {
        LNBD = 1; // so we reset to zero at each row because the assumption is the FRAME is the parent of all the obstalces inside the image!
        for(int col = 1 ; col < working_grid_.at(row).size() - 1 ; col++ ){

            int current_cell_state_,left_cell_state_,right_cell_state_;
            current_cell_state_ = working_grid_[row][col];
            if(col > 0)
                left_cell_state_ = working_grid_[row][col-1];
            if(col < working_grid_.at(row).size() - 1)
                right_cell_state_ = working_grid_[row][col+1];

            // We are tracking the free space
            // This is outer border cell scenario (already we are on freespace and the left cell is obstalce)
            if (current_cell_state_ == 1 && 
                left_cell_state_ == 0) {
                contour_count_++;
                Contour c;
                c.id_ = contour_count_;
                c.is_hole_ = false;
                c.parent_id_ = LNBD;
                c.points_.push_back(Point<int>{col,row}) ;
                trace(c,row, col, row, col-1, Point<int>{col,row}, std::nullopt);
                contours_.push_back(c);
            }
            // This is the hole scenraio (we are already on obstalce and the right side is free)
            // If we are on obstalce and both left and right are free then the first if above has the priority
            // hole detection must also work when the current pixel was already labeled. so use current>=1
            else if(current_cell_state_ >= 1 &&
                right_cell_state_ == 0) {
                contour_count_++;
                Contour c;
                c.id_ = contour_count_;
                c.is_hole_ = true;
                c.parent_id_ = LNBD;
                c.points_.push_back(Point<int>{col,row}) ;
                // working_grid_[row][col] = c.id_;
                trace(c,row, col, row, col+1, Point<int>{col,row}, std::nullopt);
                contours_.push_back(c);
            }
            if (std::abs(working_grid_[row][col]) > 1) LNBD = std::abs(working_grid_[row][col]); // working_grid_ changes in the trace function 

        }
    }
}



    /**
        feel like i should use another struct for indinces that uses int instead of point which is in double or make point template!

        about lnbd:
    * LNBD = Last New Border Descriptor.
    *
    * While scanning one row from left to right, LNBD stores the ID of the
    * most recent already-known border that is relevant to the current pixel.
    *
    * It is NOT automatically the parent of a newly discovered contour.
    *
    * LNBD is used to determine the parent by comparing:
    *
    *     1. the type of the NEW contour
    *     2. the type of the LNBD contour
    *
    * Border types:
    *
    *     outer contour  -> is_hole_ == false
    *     hole contour   -> is_hole_ == true
    *
    *
    * Parent rule:
    *
    *     +----------------+----------------+----------------------+
    *     | New contour    | LNBD contour   | Parent of new       |
    *     +----------------+----------------+----------------------+
    *     | outer          | outer          | parent(LNBD)         |
    *     | outer          | hole           | LNBD                 |
    *     | hole           | outer          | LNBD                 |
    *     | hole           | hole           | parent(LNBD)         |
    *     +----------------+----------------+----------------------+
    *
    *
    * Equivalent rule:
    *
    *     if new contour and LNBD contour have DIFFERENT types:
    *
    *         parent(new) = LNBD
    *
    *     if new contour and LNBD contour have the SAME type:
    *
    *         parent(new) = parent(LNBD)
    *
    *
    * Intuition:
    *
    *     Different type:
    *
    *         outer -> hole
    *         hole  -> outer
    *
    *     means the new contour is nested directly inside LNBD.
    *
    *
    *     Same type:
    *
    *         outer -> outer
    *         hole  -> hole
    *
    *     means the two contours are siblings, so the new contour gets
    *     the same parent as LNBD.
    *
    *
    * Example:
    *
    *         outer contour ID 2
    *         ├── hole ID 3
    *         └── hole ID 4
    *
    *     When discovering hole 4, LNBD may be hole 3.
    *
    *         new.is_hole_  = true
    *         LNBD.is_hole_ = true
    *
    *     Same type, therefore:
    *
    *         parent(4) = parent(3) = 2
    *
    *     NOT:
    *
    *         parent(4) = 3
    *
    *
    * Special case:
    *
    *     If LNBD == 1 and contour 1 is your artificial image/frame border,
    *     then the new top-level contour gets:
    *
    *         parent_id_ = 1


    While scanning the current row from left to right, LNBD remembers the border ID of the last labeled border pixel you passed.
    so the last border encountered during THIS left-to-right row scan --> That information helps Suzuki determine the hierarchy when a new border begins.
    Example:
    scan direction --->

    0 0  2 2 2  1 1 1  3 3  1 1
        ^                  ^
    border 2           border 3

    As you scan:
    before reaching border 2:
    LNBD = 1        // artificial frame

    after passing a pixel labeled ±2:
    LNBD = 2

    later, after passing a pixel labeled ±3:
    LNBD = 3


When creating a new contour:
new contour type = c.is_hole_
LNBD contour type = contours_[LNBD - 1].is_hole_

Parent rules:
new outer + LNBD outer -> parent = parent(LNBD)

new outer + LNBD hole  -> parent = LNBD

new hole  + LNBD outer -> parent = LNBD

new hole  + LNBD hole  -> parent = parent(LNBD)

Compact rule:
if (c.is_hole_ != lnbd_contour.is_hole_)
    c.parent_id_ = LNBD;
else
    c.parent_id_ = lnbd_contour.parent_id_;

Special case:
if (LNBD == 1)
    c.parent_id_ = 1;



    */

void MapGeometry::trace(Contour& c, int row, int col , int prev_row, int prev_col, const Point<int>& start_pixel, std::optional<Point<int>> first_successor){
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
        
        if (neighbor_index_.y_ >= static_cast<int>(working_grid_.size()) ||
            neighbor_index_.y_ < 0 ||
            neighbor_index_.x_ >= static_cast<int>(working_grid_.at(0).size()) ||
            neighbor_index_.x_ < 0)
            continue;

        int neighbor_state_ = working_grid_[neighbor_index_.y_][neighbor_index_.x_];

        /*
            1       -> unvisited foreground, can continue
            c.id_   -> already visited by THIS contour, can be used to close the loop
            other ID -> belongs to another contour, do not follow it
            0       -> background
        */
        if (neighbor_state_ == 1 || std::abs(neighbor_state_) == c.id_){
            if (first_successor.has_value()) {
                if (current_index_ == start_pixel  && neighbor_index_ == first_successor.value()){
                    std::cout<<"FINISHED TRACING \n";
                    return;
                } 
            }
            found_neighbor = true;
            if(!first_successor.has_value()){
                first_successor = Point<int>{neighbor_index_.x_,neighbor_index_.y_};
            }

            // Suzuki-Abe labeling rule for CURRENT pixel
            if (working_grid_[current_index_.y_][current_index_.x_ + 1] == 0)
            {
                working_grid_[current_index_.y_][current_index_.x_] = -c.id_;
            }
            else if (working_grid_[current_index_.y_][current_index_.x_] == 1)
            {
                working_grid_[current_index_.y_][current_index_.x_] = c.id_;
        }

            c.points_.push_back({current_index_});
            previous_index = current_index_;
            current_index_ = neighbor_index_;        
            break;
        }
    }
    if(!found_neighbor){
        std::cout<< "No Next border pixel found \n";
        return;
    }
    trace(c,current_index_.y_,current_index_.x_, previous_index.y_ , previous_index.x_, start_pixel, first_successor);
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


void MapGeometry::listContours(){
    if(contours_.empty())
        std::cout<<"No Contour\n";

    for(const auto& c : contours_){
        std::cout<<"==================\n";
        std::cout<<"Contour id:" <<c.id_<<"\n";
        std::cout<<"Contour is_hole:" <<c.is_hole_<<"\n";
        std::cout<<"Contour parent_id_:" <<c.parent_id_<<"\n";
        std::cout<<"Contour Points [row,col]: \n";
        for(const auto& p : c.points_){
            std::cout<<"["<<p.y_<<","<<p.x_<<"]"<<"\n";
        }
        std::cout<<"==================\n";
    }
}