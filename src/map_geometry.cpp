#include "cleaner/map_geometry.hpp"
#include "cleaner/occupancy_grid.hpp"
#include <array>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <optional>


/**
 * Build the binary working image used by Suzuki-Abe.
 *
 *      1 = foreground = traversable free space
 *      0 = background = occupied / unknown / non-traversable
 *
 * A one-cell background padding is added around the original map so
 * contour tracing can inspect all 8 neighbors without the foreground
 * touching the array boundary.
 *
 * Border ID 1 is reserved as a synthetic hierarchy root.
 * Real detected borders therefore start at ID 2.
 */
/*
 * Outer border:
 *   The border between a foreground (1) connected component
 *   and the background/0-component that directly surrounds it.
 *
 * Hole border:
 *   The border between a hole (an enclosed 0-component)
 *   and the foreground (1-component) that directly surrounds that hole.
 *
 * Important:
 *   BOTH outer borders and hole borders are represented by FOREGROUND pixels.
 */

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

/**
 * Suzuki-Abe only cares about binary image semantics:
 *
 *      1 = foreground
 *      0 = background
 *
 * The algorithm itself does NOT care whether foreground means
 * "obstacle" or "free space".
 *
 *
 * Previous interpretation:
 *
 *      1 = obstacle
 *      0 = free space
 *
 * Then:
 *
 *      current == 1 && left == 0
 *          -> start of an OUTER obstacle contour
 *
 *      current >= 1 && right == 0
 *          -> possible HOLE contour inside the obstacle foreground
 *
 *
 * For BCD we now choose:
 *
 *      1 = free space
 *      0 = obstacle / unknown / non-traversable
 *
 * The Suzuki-Abe conditions stay exactly the same:
 *
 *      current == 1 && left == 0
 *          -> start of an OUTER free-space contour
 *
 *      current >= 1 && right == 0
 *          -> start of a HOLE contour in the free-space foreground
 *
 * In this interpretation, such a hole usually represents an obstacle
 * completely enclosed by free space.
 *
 *
 * So:
 *
 *      OLD:
 *          foreground = obstacles
 *          outer contour = obstacle boundary
 *          hole contour  = free-space hole inside obstacle region
 *
 *      NEW:
 *          foreground = free space
 *          outer contour = free-space boundary
 *          hole contour  = obstacle enclosed by free space
 *
 *
 * The contour-tracing algorithm does not change.
 * Only the semantic meaning of "foreground" changes.
 */

 /**
 * Suzuki-Abe border detection and hierarchy construction.
 *
 * The image is scanned row-by-row from left to right.
 *
 * NBD:
 *      ID assigned to each newly discovered border.
 *
 * LNBD:
 *      ID of the most recently encountered known border on the
 *      current scanline. It is reset to the synthetic root (1)
 *      at the beginning of every row.
 *
 * New border conditions:
 *
 *      current == 1 && left == 0
 *          -> new OUTER border of the foreground
 *
 *      current >= 1 && right == 0
 *          -> new HOLE border
 *
 * For a hole start, if current > 1, the current positive border
 * label becomes LNBD before hierarchy assignment.
 *
 * Parent selection:
 *
 *      new type != LNBD type
 *          -> parent(new) = LNBD
 *
 *      new type == LNBD type
 *          -> parent(new) = parent(LNBD)
 *
 * In this implementation ID 1 is a synthetic root, so a top-level
 * contour whose LNBD is 1 receives parent_id = 1.
 *
 * After processing each pixel, a labeled pixel updates:
 *
 *      LNBD = abs(pixel_label)
 *
 * because the sign contains border-state information while the
 * absolute value identifies the border.
 */

void MapGeometry::findContours(){
    int contour_count_ = 1; // assuming frame of the picture is the first countor    
    int LNBD = 1; // the contours vector indices starts with 1 which is the frame index --> this variable is for tracking the border to assign parents in a hierarchy
    for(int row = 1 ; row < working_grid_.size() - 1 ; row++)
    {
        // LNBD: “What boundary was the last one I crossed before reaching this new boundary?”
        LNBD = 1; // so we reset to one at each row because the assumption is the FRAME is the parent of all the obstalces inside the image!
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
                /*
                    Parent rule:

                    new type != LNBD type
                        => new contour is directly inside LNBD
                        => parent = LNBD

                    new type == LNBD type
                        => they are siblings at the same nesting level
                        => parent = parent(LNBD)


                    one caveat! with my current represenation i need to take care of a special case of LNBD==1
                    because Take the very first real free-space contour:
                    frame:       is_hole = false
                    new contour: is_hole = false
                    parent(new) = parent(LNBD);
                    You get:
                        frame 1        parent -1
                        outer 2        parent -1   // sibling of frame
                    But you want:
                        frame 1
                        └── outer 2

                
                */
                if (LNBD == 1)
                    c.parent_id_ = 1;
                else if (c.is_hole_ == contours_.at(LNBD-1).is_hole_)
                    c.parent_id_ = contours_.at(LNBD-1).parent_id_;
                else
                    c.parent_id_ = contours_.at(LNBD-1).id_;
                trace(c,row, col, row, col-1);
                contours_.push_back(c);
            }
            // current pixel belongs to free-space foreground,
            // and the pixel to the right is background/non-traversable.
            // This is the hole-border starting condition.
            // hole detection must also work when the current pixel was already labeled. so use current>=1
            else if(current_cell_state_ >= 1 &&
                right_cell_state_ == 0) {

                if (current_cell_state_ > 1)
                    LNBD = current_cell_state_;

                contour_count_++;
                Contour c;
                c.id_ = contour_count_;
                c.is_hole_ = true;

                if (LNBD == 1)
                    c.parent_id_ = 1;
                else if (c.is_hole_ == contours_.at(LNBD-1).is_hole_)
                    c.parent_id_ = contours_.at(LNBD-1).parent_id_;
                else
                    c.parent_id_ = contours_.at(LNBD-1).id_;

                // working_grid_[row][col] = c.id_;
                trace(c,row, col, row, col+1);
                contours_.push_back(c);
            }
            /*
                why update in the end?
                Because LNBD is supposed to describe the border encountered before the current newly discovered border.
                At column col, parent selection should use the history from pixels to the left:
            */
            if (std::abs(working_grid_[row][col]) > 1) LNBD = std::abs(working_grid_[row][col]); // working_grid_ changes in the trace function 

        }
    }
}


/**
 * Follow one Suzuki-Abe border and label its pixels.
 *
 * INPUT
 * -----
 * c:
 *      The contour currently being traced.
 *      c.id_ is the current NBD (border ID).
 *
 * row, col:
 *      Starting FOREGROUND pixel of the newly detected border.
 *
 * prev_row, prev_col:
 *      Initial BACKGROUND reference pixel used to determine
 *      the side from which the border following begins.
 *
 *      Outer border -> reference is left of start pixel.
 *      Hole border  -> reference is right of start pixel.
 *
 *
 * OUTPUT / SIDE EFFECTS
 * ---------------------
 * 1. c.points_
 *      Receives the ordered foreground pixels belonging to this border.
 *
 * 2. working_grid_
 *      Border pixels are relabeled with +NBD or -NBD.
 *
 *
 * ================================================================
 * STAGE 1 — INITIAL CLOCKWISE SEARCH  (Suzuki step 3.1)
 * ================================================================
 *
 * Input:
 *      start_pixel + initial background reference
 *
 * Search clockwise around start_pixel until the first NONZERO
 * foreground neighbor is found.
 *
 * Output:
 *      first_neighbor
 *
 * IMPORTANT:
 *      first_neighbor is NOT immediately the next traced pixel.
 *      It becomes the reference direction for the first real
 *      counter-clockwise border search.
 *
 * If no nonzero neighbor exists, the start pixel is an isolated
 * one-pixel component:
 *
 *      label it -NBD
 *      store it in c.points_
 *      return
 *
 *
 * ================================================================
 * STAGE 2 — INITIALIZE BORDER FOLLOWING  (Suzuki step 3.2)
 * ================================================================
 *
 * Set:
 *
 *      reference = first_neighbor
 *      current   = start_pixel
 *
 * Meaning:
 *
 *      current   = border pixel that we are processing now
 *      reference = neighbor telling us where the CCW search starts
 *
 *
 * ================================================================
 * STAGE 3 — FIND NEXT BORDER PIXEL CCW  (Suzuki step 3.3)
 * ================================================================
 *
 * Input:
 *      current + reference
 *
 * Starting immediately after reference, inspect the 8 neighbors
 * of current in COUNTER-CLOCKWISE order.
 *
 * Stop at the first NONZERO pixel.
 *
 * Output:
 *      next = next foreground border pixel
 *
 * While examining neighbors, remember whether the pixel directly
 * to the RIGHT of current:
 *
 *      (current.x + 1, current.y)
 *
 * was actually examined and was zero.
 *
 * This information is needed by the signed NBD labeling rule.
 *
 *
 * ================================================================
 * STAGE 4 — STORE AND LABEL CURRENT PIXEL  (Suzuki step 3.4)
 * ================================================================
 *
 * Add current to c.points_.
 *
 * Then label current:
 *
 *      if right-hand zero was examined:
 *          current = -NBD
 *
 *      else if current is still 1:
 *          current = +NBD
 *
 *      else:
 *          preserve its existing border label
 *
 * Meaning of the sign:
 *
 *      abs(value) = border ID
 *
 *      -NBD means the zero-region on the right was encountered
 *      while following this border.
 *
 * The sign is later used during raster scanning to prevent an
 * already-followed hole border from being detected again.
 *
 *
 * ================================================================
 * STAGE 5 — CLOSE OR ADVANCE THE BORDER  (Suzuki step 3.5)
 * ================================================================
 *
 * The border is finished when:
 *
 *      current == first_neighbor
 *      AND
 *      next == start_pixel
 *
 * This means the tracing process has returned to its initial
 * directed border configuration.
 *
 * Otherwise advance:
 *
 *      reference = current
 *      current   = next
 *
 * and repeat stages 3-5.
 *
 *
 * HIGH-LEVEL FLOW
 * ---------------
 *
 * start pixel
 *      |
 *      | clockwise bootstrap
 *      v
 * first_neighbor
 *      |
 *      | becomes reference, NOT current
 *      v
 * current = start
 *
 *      repeat:
 *          CCW search -> next
 *          label/store current
 *          check closure
 *          reference = current
 *          current   = next
 */

void MapGeometry::trace( Contour& c, int row, int col, int prev_row, int prev_col)
{
    Point<int> current_pixel_{col,row};
    Point<int> initial_ref_{prev_col, prev_row};
    // finding the first neighbor using CW movement from the initial_ref_
    Point<int> initial_dir_ = initial_ref_ - current_pixel_;
    auto start = std::find(clockwise_.begin() , clockwise_.end() , initial_dir_);
    int start_index_ = static_cast<int>(start-clockwise_.begin());
    Point<int> first_neighbor_;
    bool first_neighbor_found_ = false;
    // CW movement --> this is initial first neighbor finding
    for(int i = start_index_; i < start_index_ + 8 ; i++)
    {
        int current_index_ = (i+1)%8;
        Point<int> step = clockwise_.at(current_index_);
        Point<int> neighbor_cell_{ col + step.x_ , row + step.y_};
        if (working_grid_[neighbor_cell_.y_][neighbor_cell_.x_] != 0){
            first_neighbor_ = neighbor_cell_;
            first_neighbor_found_ = true;
            break;
        }
    }
    //isolated-pixel case (If the initial CW search finds no nonzero neighbor)
    if(!first_neighbor_found_){
        working_grid_[row][col] = -c.id_;
        c.points_.push_back(current_pixel_);
        c.boundary_sides_.push_back(
            {true, true, true, true}
        );


        return;
    }


    Point<int> reference_ = first_neighbor_;
    Point<int> current_ = current_pixel_;
    Point<int> next;

    // now start the trace using CCW movement (so note that initial neighbor we found is not the next node but only a reference for the start of the movement)
    while(true){


        // now we go CCW
        Point<int> dir_ = reference_ - current_;
        auto start = std::find(clockwise_.begin(), clockwise_.end() , dir_);
        int start_index_ = static_cast<int>(start - clockwise_.begin());
        bool zero_right_hand_exist = false;
        bool make_minus_ = false;
        Point<int> right_hand_background_pixel_; // this is a cell with 0 value which is on the right of the current pixel
        if(working_grid_[current_.y_][current_.x_+1] == 0){
            right_hand_background_pixel_ = Point{current_.x_+1,current_.y_}; // this is a cell with 0 value which is on the right of the current pixel
            zero_right_hand_exist = true;
        }



        // USEFULL FOR POLYGONIZATION
        /*
            Meaning:
            if top_boundary is true : the TOP edge of C belongs to this contour
            if right_boundary is true: the RIGHT edge of C belongs to this contour
        */
        bool left_boundary   = false;
        bool top_boundary    = false;
        bool right_boundary  = false;
        bool bottom_boundary = false;
        /*
            How to do CCW movement on a CW array?
            so imagine start index is 4 then 
            i separate the loop with using normal int i = 0 to less than 8 and the array indexing
            so 
            i = 0,1,2,3,4,5,6,7
            now my result need to be 
            3,2,1,0,-1,-2,-3,-4
            (well technically it needs to be this but i solve wrapping afterward:) 3,2,1,0,7,6,5,4

            so using start index - 1 , start_index_ - 2 ....., start_index_-8 i get  the above numbers so it would be 
            j = (start_index_-(i+1))
            so now i also need to fix the wrapping problem
            which is something modulo 8 but since negative modulo is weird in c++ lets do:
            (j+8)%8!
        */
        for(int i = 0 ; i < 8 ; i++){
            //CCW movement 
            int neighbor_index_ = ((start_index_ - (i+1)) + 8)%8; // CCW movement index in the clockwise_ array
            auto step = clockwise_.at(neighbor_index_);
            auto current_neighbor_ = current_ + step; // next CCW neighbor


            if(zero_right_hand_exist &&  current_neighbor_ == right_hand_background_pixel_)
                make_minus_ = true;

            auto state = working_grid_[current_neighbor_.y_][current_neighbor_.x_];

            ////// some rich information needed for polygonization and unnecessery for findContour///////
            // If THIS neighbor was actually examined as background,
            // remember cardinal sides for this particular border traversal.
            if (state==0){
                // where is this neighbor wrt to current_
                if(step.x_ == 1 && step.y_ == 0) {
                    // EAST
                    right_boundary = true;
                }
                else if(step.x_ == -1 && step.y_ == 0) {
                    // WEST
                    left_boundary = true;
                }
                else if(step.x_ == 0 && step.y_ == 1) {
                    // SOUTH
                    bottom_boundary = true;

                }
                else if(step.x_ == 0 && step.y_ == -1) {
                    // NORTH
                    top_boundary = true;
                }
            }
            //////////////////////////////////////////////////////////////////////////////////////////////

            if(state != 0){ // foreground test must be != 0, not >= 1. Negative -NBD pixels are still foreground.
                next = Point<int>{current_neighbor_.x_ , current_neighbor_.y_};
                // seems like we found the next cell
                // we put the current cell for now!
                c.points_.push_back(current_);

                if(make_minus_)
                 //but “Does that 0 belong to the zero-region whose boundary I am currently following?”
                /*
                    If the search actually passes through that right-hand 0, then Suzuki knows:
                    That 0 is on the background/hole side
                    of the border I am currently tracing.
                    Then Suzuki marks:
                    C = -NBD;

                    The negative sign means roughly:
                    "This border pixel has the zero-component
                    I'm currently tracing on its right side."
                */

                    working_grid_[current_.y_][current_.x_] = -c.id_;
                else if (working_grid_[current_.y_][current_.x_] == 1)
                    working_grid_[current_.y_][current_.x_] = c.id_;


                //rich info storing : unnecessary for findContour
                /*
                * boundary_sides_[i] corresponds to points_[i].
                *
                * A true side means that during THIS contour traversal,
                * that cardinal background neighbor was examined before
                * the next foreground border pixel was found.
                *
                * Therefore that side of the current cell belongs to
                * this particular contour boundary.
                */
                c.boundary_sides_.push_back({left_boundary,top_boundary,right_boundary,bottom_boundary}) ;


                // stopping condition
                if(current_ == first_neighbor_ && next == current_pixel_) return;

                reference_ = current_;
                current_ = current_neighbor_;

                break;
            }
        }

    }


}



// Recursion
void MapGeometry::trace( Contour& c, int row, int col, int prev_row, int prev_col, const Point<int>& start_pixel, std::optional<Point<int>> first_neighbor)
{
    Point<int> current_index_{col,row};
    Point<int> previous_index{prev_col,prev_row};

    /*
        first call:
            previous_index = initial background reference
            first_neighbor = empty

        recursive calls:
            previous_index = previous border pixel / search reference
            first_neighbor = Suzuki's (i1,j1), kept for stopping condition
    */


    // ---------------------------------------------------------
    // STEP 3.1 + 3.2
    // ONLY ON THE FIRST CALL:
    // find first_neighbor CLOCKWISE from the initial reference.
    // first_neighbor becomes the reference; current stays at start_pixel.
    // ---------------------------------------------------------
    if(!first_neighbor.has_value())
    {
        Point<int> dir = previous_index - current_index_;
        auto start = std::find(
            clockwise_.begin(),
            clockwise_.end(),
            dir);

        if(start == clockwise_.end())
            return;

        int start_index_ = static_cast<int>(start - clockwise_.begin());

        bool found_neighbor = false;

        for(int i = start_index_; i < start_index_ + 8; i++) {
            // CLOCKWISE
            int current_neighbor_index_ = (i+1)%8;

            Point<int> step = clockwise_.at(current_neighbor_index_);

            Point<int> neighbor_index_{ col + step.x_, row + step.y_ };

            if (neighbor_index_.y_ >= static_cast<int>(working_grid_.size()) ||
                neighbor_index_.y_ < 0 ||
                neighbor_index_.x_ >= static_cast<int>(working_grid_.at(0).size()) ||
                neighbor_index_.x_ < 0)
                continue;

            int neighbor_state_ = working_grid_[neighbor_index_.y_][neighbor_index_.x_];

            if(neighbor_state_ != 0)
            {
                first_neighbor = neighbor_index_;
                found_neighbor = true;
                break;
            }
        }


        // isolated foreground pixel:
        // no nonzero pixel exists in its 8-neighborhood
        if(!found_neighbor)
        {
            working_grid_[current_index_.y_][current_index_.x_] = -c.id_;
            c.points_.push_back(current_index_);
            return;
        }


        /*
            Suzuki step 3.2:

                reference = first_neighbor
                current   = start_pixel

            IMPORTANT:
            we DO NOT move current to first_neighbor.
            first_neighbor is only the reference for the first CCW search.
        */
        previous_index = first_neighbor.value();
    }



    // ---------------------------------------------------------
    // STEP 3.3
    // Search CCW around current, starting after previous_index.
    // ---------------------------------------------------------

    Point<int> dir = previous_index - current_index_;
    auto start = std::find( clockwise_.begin(), clockwise_.end(), dir);
    if(start == clockwise_.end())
        return;

    int start_index_ = static_cast<int>(start - clockwise_.begin());

    bool found_neighbor = false;
    bool right_zero_examined = false;

    Point<int> next_index_;


    for(int i = 0; i < 8; i++)
    {
        /*
            clockwise_ is stored:

            E, SE, S, SW, W, NW, N, NE

            To move CCW we go backward through the array:

            start-1, start-2, ..., start-8
        */
        int current_neighbor_index_ = (start_index_ - (i+1) + 8)%8;

        Point<int> step = clockwise_.at(current_neighbor_index_);

        Point<int> neighbor_index_{ col + step.x_, row + step.y_ };

        if (neighbor_index_.y_ >= static_cast<int>(working_grid_.size()) ||
            neighbor_index_.y_ < 0 ||
            neighbor_index_.x_ >= static_cast<int>(working_grid_.at(0).size()) ||
            neighbor_index_.x_ < 0)
            continue;

        int neighbor_state_ = working_grid_[neighbor_index_.y_][neighbor_index_.x_];


        /*
            Suzuki step 3.4(a):

            If the CCW search actually examines the 0-pixel directly
            to the right of current, remember it.

            right pixel = (current.x + 1, current.y)

            This means that this zero-component is on the right side
            of the border currently being followed, so current will
            later receive -NBD.
        */
        if(neighbor_index_.x_ == current_index_.x_ + 1 &&
           neighbor_index_.y_ == current_index_.y_ &&
           neighbor_state_ == 0) {
            right_zero_examined = true;
        }


        // first nonzero neighbor is the next border pixel
        if(neighbor_state_ != 0) {
            next_index_ = neighbor_index_;
            found_neighbor = true;
            break;
        }
    }


    if(!found_neighbor) {
        std::cout << "No Next border pixel found\n";
        return;
    }



    // ---------------------------------------------------------
    // STEP 3.4
    // Store and label CURRENT pixel.
    // ---------------------------------------------------------

    c.points_.push_back(current_index_);


    if(right_zero_examined) {
        working_grid_[current_index_.y_][current_index_.x_] = -c.id_;
    }
    else if(working_grid_[current_index_.y_][current_index_.x_] == 1) {
        working_grid_[current_index_.y_][current_index_.x_] = c.id_;
    }



    // ---------------------------------------------------------
    // STEP 3.5
    //
    // finish when:
    //
    // current == first_neighbor
    // next    == start_pixel
    // ---------------------------------------------------------

    if(current_index_ == first_neighbor.value() &&
       next_index_ == start_pixel)
    {
        std::cout << "Finished Tracing Contour : " << c.id_ << "\n";
        return;
    }

    /*
        Recursive version of:

            reference = current;
            current   = next;
    */
    trace(c, next_index_.y_, next_index_.x_, current_index_.y_, current_index_.x_, start_pixel, first_neighbor);
}





void MapGeometry::showContours(const cv::Mat& original_image)
{
    cv::Mat colored_image;
    cv::cvtColor(original_image, colored_image, cv::COLOR_GRAY2BGR);

    for (const auto& c : contours_)
    {
        for (const auto& p : c.points_)
        {
            // Contours are stored in padded working_grid_ coordinates.
            // Original image coordinates are shifted by (-1, -1).
            const int x = p.x_ - 1;
            const int y = p.y_ - 1;

            // Safety check before accessing cv::Mat.
            if (x < 0 || x >= colored_image.cols ||
                y < 0 || y >= colored_image.rows)
            {
                continue;
            }

            colored_image.at<cv::Vec3b>(y, x) =
                cv::Vec3b(0, 0, 255);
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
        // for(const auto& p : c.points_){
        //     std::cout<<"["<<p.y_<<","<<p.x_<<"]"<<"\n";
        // }
        std::cout<<"==================\n";
    }
}



/**
 * Polygonization:
 *
 * findContours()/trace() already determined, for every contour cell,
 * which sides of that cell belong to THIS particular contour.
 *
 * Therefore:
 *
 * for each real Contour
 *      ↓
 * for each points_[i]
 *      ↓
 * read boundary_sides_[i]
 *      ↓
 * for every side marked true:
 *      generate the corresponding grid-edge Segment
 *      ↓
 * later: connect/order the segments into a closed polygon
 *
 *
 * Grid geometry convention:
 *
 * cell [row][col] occupies:
 *
 * (col,row) -------- (col+1,row)
 *     |                    |
 *     |        CELL        |
 *     |                    |
 * (col,row+1) ---- (col+1,row+1)
 *
 *
 * Directed edges are stored clockwise around the free cell:
 *
 * top:
 *      (col,row) -> (col+1,row)
 *
 * right:
 *      (col+1,row) -> (col+1,row+1)
 *
 * bottom:
 *      (col+1,row+1) -> (col,row+1)
 *
 * left:
 *      (col,row+1) -> (col,row)
 *
 * NOTE:
 * These are GRID-VERTEX coordinates, not world coordinates yet.
 */

void MapGeometry::polygonize()
{
    if (contours_.empty()) {
        std::cout << "No Contours!\n";
        return;
    }

    crack_boundaries_.clear();
    crack_boundaries_.resize(contours_.size());

    for (const auto& c : contours_)
    {
        // Synthetic Suzuki hierarchy root has no real boundary.
        if (c.id_ == 1 || c.points_.empty())
            continue;

        crack_boundaries_[c.id_ - 1] = traceCrackBoundary(c);
    }
}
/*
    What traceCrackBoundary() does
    It is another tracer, like Suzuki, except its state is:
    current grid vertex
    current direction

    rather than:
    current foreground cell
    reference foreground cell


    ALGORITHM:
        1. Get Suzuki start cell.

        2. Determine starting crack: (convention : while walking on the crack (Your POV should be on the crack), free space stays on the right side.)
            outer -> left edge, moving UP
            hole  -> right edge, moving DOWN

        3. Now forget the Suzuki points temporarily.

        4. At the end vertex of the current crack:
            examine the four possible grid-edge directions
            N, E, S, W.

        5. A candidate edge is valid when:
            cell on LEFT  = background (0)
            cell on RIGHT = foreground (!=0)

        6. Choose the valid next edge.

        7. Add its next grid vertex to the polygon.

        8. Repeat until the starting directed edge is reached again.

*/

std::vector<Segment<int>> MapGeometry::traceCrackBoundary(const Contour& c){
    std::vector<Segment<int>> segs_;

    /*
        convention is that the topleft vertex of the cell is P = (col , row)
        so:

        OUTER contour: background is LEFT of p

        0 | FREE
        ↑

        start vertex = (col, row+1)
        end vertex   = (col, row)
        direction    = NORTH

        and:
        For a hole:
        FREE | 0
            ↓

        start vertex = (col+1, row)
        end vertex   = (col+1, row+1)
        direction    = SOUTH
    
    */
    // IMPORTANT: we are working with grid points now (which is the 4 points around the rectangular cell! for convention assume the cell index is the top-left and treat it as grid point)
    Point<int> front_point_ = c.points_.front(); // we use this and the is_hole to decide whats the start_ and end_ of the first segement is!
    Point<int> dir;
    Point<int> start_;
    Point<int> end_;
    if (!c.is_hole_) { //outer border
        start_ = Point<int>{front_point_.x_ , front_point_.y_ + 1};
        end_ = Point<int>{front_point_.x_ , front_point_.y_};
        dir = Point<int>{0,-1}; //North

    }
    else{
        start_ = Point<int>{front_point_.x_ + 1 , front_point_.y_};
        end_ = Point<int>{front_point_.x_ + 1 , front_point_.y_ + 1};
        dir = Point<int>{0,1}; //South
    }
    segs_.push_back({start_,end_});
    // Till now the state is : current vertex and current direction! --> so the current vertex is end_ and current direction thus far is either north or south!
    /*
        At each vertex, try directions relative to the direction you arrived from:
        evaluate E/S/W/N
        count valid crack edges

        1 valid -> follow it
        0 valid -> error
        2 valid -> diagonal ambiguity
        Why this order? Because my convention is:
        background stays on the LEFT, free space stays on the RIGHT. so when we deal with outer border we choose the direction
        of north so the left would be background(obstalce or 0) and right of the edge would be foreground(freespace or 1) so the convention we choose in suzuku is satisfied
        
        mind taht imagine you are walking on the directed edge so your POV should be in the edge for choosing direction


        For each candidate direction, ask only:
        Does this grid edge have:

            LEFT cell  == 0
            RIGHT cell != 0
        ?

    */

    // now ask can you go N?E?S?W?

    Point<int> current_vertex_ = end_;
    /*
        current vertex is shared by four cells:
                        x-1        x

                    +-----------+-----------+
                    |           |           |
            y-1     |    NW     |    NE     |
                    |           |           |
                    +-----------X-----------+  <- current vertex (x,y)
                    |           |           |
            y       |    SW     |    SE     |
                    |           |           |
                    +-----------+-----------+
    
    */

    // For each possible directed edge, you need to inspect the two cells on the two sides of that edge.
    while(true){
    /**
    * Crack-edge convention:
    *
    * "left" and "right" are relative to the WALKING DIRECTION, not the image.
    * A valid directed crack edge always satisfies:
    *
    *      LEFT side  = background / obstacle = 0
    *      RIGHT side = foreground / free    != 0
    *
    * Around current vertex (x,y):
    *
    *          NW | NE
    *             |
    *             X
    *             |
    *          SW | SE
    *
    * Direction table:
    *
    *      EAST  (x,y) -> (x+1,y) : left = NE , right = SE
    *          valid if NE == 0 && SE != 0
    *
    *      SOUTH (x,y) -> (x,y+1) : left = SE , right = SW
    *          valid if SE == 0 && SW != 0
    *
    *      WEST  (x,y) -> (x-1,y) : left = SW , right = NW
    *          valid if SW == 0 && NW != 0
    *
    *      NORTH (x,y) -> (x,y-1) : left = NW , right = NE
    *          valid if NW == 0 && NE != 0
    *
    * If a direction is valid:
    *
    *      next_vertex = current_vertex + direction
    *
    * Summary:
    *      E -> NE / SE
    *      S -> SE / SW
    *      W -> SW / NW
    *      N -> NW / NE
    *
    * The rule is always the same: background on the left, free space on the right.
    */
        int NE = working_grid_[current_vertex_.y_ - 1][current_vertex_.x_]; // you go north from the current vertex to reach the top left corenr of the NE cell
        int SE = working_grid_[current_vertex_.y_][current_vertex_.x_];
        int SW = working_grid_[current_vertex_.y_][current_vertex_.x_ - 1];
        int NW = working_grid_[current_vertex_.y_ - 1][current_vertex_.x_ - 1];

        int count=0;
        bool east = (NE==0 && SE != 0);  //!= 0 is important because Suzuki may have changed foreground values to +NBD or -NBD.
        if(east)
            count++;
        bool south = (SE==0 && SW != 0);
        if(south)
            count++;
        bool west = (SW==0 && NW != 0);
        if(west)
            count++;
        bool north = (NW==0 && NE != 0);
        if(north)
            count++;

        if(count==0){
            std::cout<<"ERROR \n";
            break;
        }
        else if (count==1) {
            if (east) {
                segs_.push_back({current_vertex_ , Point<int>{current_vertex_.x_+1,current_vertex_.y_}});
                current_vertex_ = Point<int>{current_vertex_.x_+1,current_vertex_.y_};
            }
            else if(south){
                segs_.push_back({current_vertex_ , Point<int>{current_vertex_.x_,current_vertex_.y_+1}});
                current_vertex_ = Point<int>{current_vertex_.x_,current_vertex_.y_+1};
            }
            else if(west){
                segs_.push_back({current_vertex_ , Point<int>{current_vertex_.x_-1,current_vertex_.y_}});
                current_vertex_ = Point<int>{current_vertex_.x_-1,current_vertex_.y_};
            }
            else if(north){
                segs_.push_back({current_vertex_ , Point<int>{current_vertex_.x_,current_vertex_.y_-1}});
                current_vertex_ = Point<int>{current_vertex_.x_,current_vertex_.y_-1};
            }
        
        }
        else {
            /*
                for example:

                FREE | 0
                -----X-----
                0  | FREE
            */
            std::cout<<"AMBIGUIOUS: TURN LEFT FOR NOW! \n";
            // for now choose LEFT turn
            // because it preserves Suzuki's 8-connected foreground semantics
            /*
                this might create a polygon that touches itself at a single vertex:
                    +---+
                    |   |
                    +---X---+
                        |   |
                        +---+
            Then, before BCD, we inspect whether the resulting polygon contains repeated vertices / point-touching geometry. If it does, we regularize or split it into BCD-safe simple polygons.
            
            */
            segs_.push_back({current_vertex_ , Point<int>{current_vertex_.x_+1,current_vertex_.y_}});
            current_vertex_ = Point<int>{current_vertex_.x_+1,current_vertex_.y_};


        }

        if(current_vertex_==start_){
            break;
        }

    }
    return segs_;

}


void MapGeometry::showCrackBoundaries(const cv::Mat& original_image)
{
    if (crack_boundaries_.empty()) {
        std::cout << "No crack boundaries. Call polygonize() first!\n";
        return;
    }

    cv::Mat display;

    if (original_image.channels() == 1)
        cv::cvtColor(original_image, display, cv::COLOR_GRAY2BGR);
    else
        display = original_image.clone();

    for (const auto& boundary : crack_boundaries_)
    {
        if (boundary.empty())
            continue;

        std::vector<cv::Point> points;
        points.reserve(boundary.size() + 1);

        /*
         * Crack coordinates are in padded working_grid_ coordinates.
         * Remove the one-cell Suzuki padding.
         *
         * No visualization scaling is done here, so the resulting image
         * has exactly the same size as showContours().
         */
        auto toImagePoint = [](const Point<int>& p)
        {
            return cv::Point{
                p.x_ - 1,
                p.y_ - 1
            };
        };

        points.push_back(toImagePoint(boundary.front().start_));

        for (const auto& seg : boundary)
            points.push_back(toImagePoint(seg.end_));

        // Draw the whole ordered crack boundary efficiently.
        cv::polylines(
            display,
            points,
            false,
            cv::Scalar(0, 0, 255),
            1,
            cv::LINE_8
        );

        /*
         * Mark the starting point so we can see where this directed
         * boundary trace began.
         */
        cv::circle(
            display,
            toImagePoint(boundary.front().start_),
            2,
            cv::Scalar(0, 255, 255),
            -1
        );
    }

    // Same behavior as showContours().
    cv::imshow("Directed Crack Boundaries", display);
    cv::waitKey(0);
}