#include "cleaner/boustrophedon_decomposer.hpp"


void BoustrophedonDecomposer::compute(const PolygonWithHoles<int>& polygon){
    vertical_slices_.clear();
    bcd_cells_.clear();
    graph_.clear();


    // 2. Collect x coordinates of outer + hole vertices.
    std::vector<int> critical_x;
    for(const auto& p   : polygon.outer_.points_) {
        critical_x.push_back(p.x_);
    }

    for(const auto& hole  : polygon.holes_){
        for(const auto& p : hole.points_ ){
            critical_x.push_back(p.x_);
        }
    }

    // 3. Sort unique x coordinates.
    std::sort(critical_x.begin(),critical_x.end(), [](int a , int b){
        return a < b;
    });
    /*
        std::unique() rearranges it conceptually into:
        1, 74, 191, 630, ?, ?, ?
                        ^
                        |
                returned iterator

        It moves one copy of every unique value toward the front and returns an iterator pointing to the first unwanted element.
        now we can erase from that retured iterator to the end
    
    */
    critical_x.erase(std::unique(critical_x.begin(), critical_x.end()),critical_x.end());

    for(const auto& x : critical_x){
        std::cout<<x<<",";
    }
    std::cout<<"\n";



    // using the mid points of consecutive x's to avoid degenerate cases of vertical x being on a vertical side of a rectangle for example
    for (int i = 0 ; i + 1 < critical_x.size() ; i++){
        VerticalSlice vertical_slice_;
        double x1 = critical_x.at(i);
        double x2 = critical_x.at(i + 1);
        double mid = (x1 + x2) / 2;
        vertical_slice_.x_ = mid;
        vertical_slice_.x_left_ = x1;
        vertical_slice_.x_right_ = x2;
        vertical_slices_.push_back(vertical_slice_);
    }

    // 4. Between consecutive x coordinates, determine the vertical FREE intervals.
    /*
        Because your sample x is between critical x-values, it will not lie directly on a polygon vertex or vertical polygon edge, which avoids most degeneracy problems.
    */


    for (auto& v : vertical_slices_){
        auto intersections =  getYIntersections(v.x_ , polygon);

        if (intersections.size() < 2)
            continue;


        double x = v.x_;
        for(int i = 0 ; i + 1 < intersections.size() ; i++) {
            bool invalid = false;
            double mid_y_ = (intersections.at(i).y_ + intersections.at(i+1).y_) / 2;
            /*
                midpoint test needs to mean:
                    midpoint is INSIDE outer
                    AND
                    midpoint is OUTSIDE every hole

                    so:
                        test midpoint

                        first:
                            is it inside outer?
                            no -> invalid

                        then:
                            is it inside any hole?
                            yes -> invalid

                        otherwise:
                            free
            */
            auto outerState = pointInPolygon(Point<double>{x,mid_y_}, polygon.outer_);
            if (outerState != PointState::INSIDE)
            {
                invalid = true;
            }
            else { 
                // finding the free intervals! I think we can take a the mid point of each intervals and test to see if its in a hole (obstalce region)
                for (const auto& hole : polygon.holes_){
                    auto holeState = pointInPolygon(Point<double>{x,mid_y_}, hole);
                    if (holeState == PointState::INSIDE || holeState == PointState::BOUNDARY)
                    {
                        invalid = true;
                        break;
                    }
                }
            }


           

            if (!invalid){
                v.free_intervals_.push_back({Point<double>{x , intersections.at(i).y_ }, Point<double>{x , intersections.at(i+1).y_}});

            }

        }

    }


    for (const auto& slice : vertical_slices_){
       std::cout<<"SLICE X =  " << slice.x_ << "\n";
       for (const auto& interval : slice.free_intervals_){
        std::cout<<"["<<interval.start_<<","<<interval.end_<<"]\n";
       }
       std::cout<<"-----------\n";
       
    }

    // 5. Compare free intervals from one slab to the next.
    /*
        Given two neighboring VerticalSlices, determine which Segment<double> intervals overlap in y.
    */


    // Initialize
    int next_cell_id_ = 0;
    // 1. At the first slice, every free interval starts a new BCD cell.
    for(int i = 0 ; i < vertical_slices_.at(0).free_intervals_.size() ; i++) {
        vertical_slices_.at(0).cell_ids_.push_back(next_cell_id_);
        next_cell_id_++;
    }



    for (int i = 0 ; i + 1 < vertical_slices_.size() ; i++) {
        auto& slice_left = vertical_slices_.at(i);
        auto& slice_right = vertical_slices_.at(i + 1);
        
        // connections_ is between the slice_left free intervals and the slice_right free intervals!
        auto connections_ = overlapGraph(slice_left, slice_right);

        /*
        * Classify the connectivity change between two consecutive vertical slices.
        *
        * Each IntervalConnection stores:
        *
        *      left interval index  ->  right interval index
        *
        * For every left interval, count how many successors it has.
        * For every right interval, count how many predecessors it has.
        *
        * Interpretation:
        *
        *      1 left  -> 1 right
        *          same BCD cell continues
        *
        *      1 left  -> many right
        *          SPLIT / IN event
        *          old cell closes and multiple new cells begin
        *
        *      many left -> 1 right
        *          MERGE / OUT event
        *          old cells close and one new cell begins
        *
        *      0 successors
        *          left-side cell terminates
        *
        *      0 predecessors
        *          new right-side cell starts
        *
        * Event classification is therefore based on the degree of each interval
        * in the overlap graph, not simply on the total number of intervals
        * in the two slices.
    */
    /*
        Example:
            successors[0] = {0};        // L0 -> R0
            successors[1] = {1,2};      // L1 -> R1,R2

            predecessors[0] = {0};      // R0 <- L0
            predecessors[1] = {1};      // R1 <- L1
            predecessors[2] = {1};      // R2 <- L1 


            successors[L]    = right intervals connected to left interval L
            predecessors[R]  = left intervals connected to right interval R
    */

        std::vector<std::vector<int>> successors(slice_left.free_intervals_.size());
        std::vector<std::vector<int>> predecessors(slice_right.free_intervals_.size());

        for (const auto& connection : connections_)
        {
            int left_ind  = connection.left_interval_;
            int right_ind = connection.right_interval_;

            successors.at(left_ind).push_back(right_ind);
            predecessors.at(right_ind).push_back(left_ind);
        }
        
        /*
            if the size of left is more than one it surely is split!
            if the size of left is zero its the end!
            but if size of left is 1 then we need to choose between continue or merge?
            You take its one successor R and ask:

            How many predecessors does R have?

            If:
            successors[L].size() = 1
            predecessors[R].size() = 1

            then:
            L ------> R

            CONTINUE

            But if:
            successors[L].size() = 1
            predecessors[R].size() > 1

            then this left interval is one branch participating in a merge:
            L0 ----\
                    \
                    ----> R
                    /
            L1 ----/
        */
        // LOGS
        for (int i = 0; i < successors.size(); i++)
        {
            const auto& left = successors.at(i);

            std::cout << "L" << i
                    << " " << slice_left.free_intervals_.at(i)
                    << " -> ";

            if (left.empty())
            {
                std::cout << "END\n";
            }
            else if (left.size() > 1)
            {
                for (int right_index : left)
                {
                    std::cout << "R" << right_index
                            << " " << slice_right.free_intervals_.at(right_index)
                            << " ";
                }

                std::cout << ": SPLIT\n";
            }
            else
            {
                // left.size() == 1
                int right_index = left.at(0);

                std::cout << "R" << right_index
                        << " " << slice_right.free_intervals_.at(right_index);

                if (predecessors.at(right_index).size() == 1)
                {
                    std::cout << " : CONTINUE\n";
                }
                else
                {
                    std::cout << " : MERGE\n";
                }
            }

            std::cout << "---------------\n";
        }



        propagateCellIDs(successors, predecessors, slice_left, slice_right, next_cell_id_);

    }


    std::cout << "\n\n========== BCD CELL IDS ==========\n";
    for (int s = 0; s < vertical_slices_.size(); s++)
    {
        const auto& slice = vertical_slices_.at(s);

        std::cout << "\nSLICE " << s
                << "   x = " << slice.x_ << "\n";

        // Important sanity check
        if (slice.cell_ids_.size() != slice.free_intervals_.size())
        {
            std::cout << "ERROR: number of cell IDs does not match "
                        "number of free intervals!\n";

            std::cout << "free_intervals = "
                    << slice.free_intervals_.size()
                    << ", cell_ids = "
                    << slice.cell_ids_.size()
                    << "\n";

            continue;
        }

        for (int i = 0; i < slice.free_intervals_.size(); i++)
        {
            std::cout << "    I" << i
                    << "  " << slice.free_intervals_.at(i)
                    << "  ---> CELL "
                    << slice.cell_ids_.at(i)
                    << "\n";
        }
    }
    std::cout << "\n==================================\n";



    /*
        every VerticalSlice
            ↓
        every free interval j

    
        x_left
        x_right

        y_top    = interval.start_.y_
        y_bottom = interval.end_.y_

        cell_id = slice.cell_ids_[j]



        We need to create the rectangular fragment!

        (x_left,y_top) -------- (x_right,y_top)
            |                       |
            |       cell_id         |
            |                       |
        (x_left,y_bottom) ----- (x_right,y_bottom)



        eventually:
            BCDCell 0
                fragment
                fragment
                fragment

            BCDCell 1
                fragment
                fragment

            BCDCell 7
                fragments_[0] = rectangle A
                fragments_[1] = rectangle B
                fragments_[2] = rectangle C
                ...

    */
    /*
        for each VerticalSlice i
            slab boundaries:
                x_left  = critical_x[i]
                x_right = critical_x[i+1]

            for each free interval j:
                y_top    = interval.start.y
                y_bottom = interval.end.y
                cell_id  = cell_ids[j]

                this defines one exact 2D piece of that BCD cell
    
    */

    for (int i = 0 ; i < vertical_slices_.size() ; i++) {
        for (int j = 0 ; j < vertical_slices_.at(i).free_intervals_.size() ; j++) {
            auto free_interval_ = vertical_slices_.at(i).free_intervals_.at(j);
            auto y_top_ = free_interval_.start_.y_;
            auto y_bottom_ = free_interval_.end_.y_;
            auto x_left_ = vertical_slices_.at(i).x_left_;
            auto x_right_ = vertical_slices_.at(i).x_right_;
            auto cell_id_ = vertical_slices_.at(i).cell_ids_.at(j);
            auto iter = std::find_if(bcd_cells_.begin(), bcd_cells_.end(), [&](const BCDCell& cell){
                return cell.id_ == cell_id_;
            });

            Polygon<double> poly_;
            poly_.points_.push_back(Point<double>{x_left_,y_top_});
            poly_.points_.push_back(Point<double>{x_right_,y_top_});
            poly_.points_.push_back(Point<double>{x_right_,y_bottom_});
            poly_.points_.push_back(Point<double>{x_left_,y_bottom_});
            if (iter == bcd_cells_.end()) {
                BCDCell bcd_cell_;
                bcd_cell_.id_ = cell_id_;
                bcd_cell_.fragments_.push_back(poly_);
                bcd_cells_.push_back(bcd_cell_);
            }
            else {
                /*
                    CELL 7 for example

                    +-------+-------+-------+
                    | frag0 | frag1 | frag2 |
                    +-------+-------+-------+

                    BCDCell 7
                        fragments_[0] = rectangle A
                        fragments_[1] = rectangle B
                        fragments_[2] = rectangle C
                
                */
                iter->fragments_.push_back(poly_);
                iter->id_ = cell_id_; // Redundant! because we already found it via std find using id_ == cell_id_
            }
        }
    }

    std::cout << "\n========== BCD CELLS ==========\n";

    for (const auto& cell : bcd_cells_)
    {
        std::cout << "\nCELL " << cell.id_
                << "   fragments = "
                << cell.fragments_.size()
                << "\n";

        for (int i = 0; i < cell.fragments_.size(); i++)
        {
            const auto& fragment = cell.fragments_.at(i);

            std::cout << "  Fragment " << i << ":\n";

            for (int j = 0; j < fragment.points_.size(); j++)
            {
                std::cout << "      P" << j
                        << " = " << fragment.points_.at(j)
                        << "\n";
            }
        }

        std::cout << "-------------------------------\n";
    }

    std::cout << "\n================================\n";

    // Unifying fragments into one polygon!

    for (auto& cell : bcd_cells_) {
        // Top boundary: left -> right
        for (const auto& rect : cell.fragments_) {
            cell.polygon_.points_.push_back(rect.points_.at(0));
            cell.polygon_.points_.push_back(rect.points_.at(1));
        }
        // Bottom boundary: right -> left
        for (auto iter = cell.fragments_.rbegin() ; iter != cell.fragments_.rend() ; ++iter ) {
            cell.polygon_.points_.push_back(iter->points_.at(2));
            cell.polygon_.points_.push_back(iter->points_.at(3));
        }

        cell.polygon_ = simplifyPolygon(cell.polygon_);
    }







    // Creating the graph cells!
    /*
        The following loop may look redundant because most cells appear in an edge and we deal with it in propagateCellID function's last stage, but it's important.
        Consider:
        one free-space cell
        no obstacles

        Then you have:
        CELL 0

        but there are zero adjacency edges.
        Without:
        graph_.addCell(0);

        the graph would incorrectly contain no node at all.
    
    */
    for (const auto& cell: bcd_cells_) {
        graph_.addCell(cell.id_);
    }


    std::cout << "\n========== CELL GRAPH ==========\n";

    graph_.print(std::cout);

    std::cout << "================================\n";

}


std::vector<Point<double>> BoustrophedonDecomposer::getYIntersections(double x, const PolygonWithHoles<int>& polygon) {
    auto outer = polygon.outer_;
    auto holes = polygon.holes_;
    std::vector<Point<double>> intersectionPoints;

    // Outer intersections
    for(int i = 0 ; i < outer.points_.size() ; i++){
        Point<int> A = outer.points_.at(i);
        Point<int> B = outer.points_.at((i+1)%outer.points_.size());
        // is line x intersecting with segment AB?
        if( (x < B.x_) != (x < A.x_) ) {
            // whats the intersection point?
            double t = (x - A.x_) / (B.x_ - A.x_); // since line x is infinite then its not like a point in polygon and i dont have to test wheter the intersection y is bigger than some point.y_!
            Point<double> intersection{x, A.y_ + t * (B.y_ - A.y_)};
            intersectionPoints.push_back(intersection) ;
        }
    }
     

    // hole intersections
    for (const auto& poly : holes){
        for(int i = 0 ; i < poly.points_.size() ; i++){
            Point<int> A = poly.points_.at(i);
            Point<int> B = poly.points_.at((i+1)%poly.points_.size());
            // is line x intersecting with segment AB?
            if( (x < B.x_) != (x < A.x_) ) {
                // whats the intersection point?
                double t = (x - A.x_) / (B.x_ - A.x_); // since line x is infinite then its not like a point in polygon and i dont have to test wheter the intersection y is bigger than some point.y_!
                Point<double> intersection{x, A.y_ + t * (B.y_ - A.y_)};
                intersectionPoints.push_back(intersection) ;
            }
        }
    }

    std::sort(intersectionPoints.begin(),intersectionPoints.end() , [](const Point<double>& A, const Point<double>& B){
        return A.y_ < B.y_;
    }  );

    return intersectionPoints;

}

void BoustrophedonDecomposer::showVerticalSlices(
    const cv::Mat& original_image)
{
    cv::Mat display;

    cv::cvtColor(
        original_image,
        display,
        cv::COLOR_GRAY2BGR
    );

    // Add bottom/right edge so grid vertices remain drawable.
    cv::copyMakeBorder(
        display,
        display,
        0, 1,
        0, 1,
        cv::BORDER_CONSTANT,
        cv::Scalar(0, 0, 0)
    );

    constexpr int scale = 4;

    cv::resize(
        display,
        display,
        cv::Size(),
        scale,
        scale,
        cv::INTER_NEAREST
    );

    for (const auto& slice : vertical_slices_)
    {
        for (const auto& interval : slice.free_intervals_)
        {
            /*
             * BCD geometry uses padded working-grid coordinates.
             * Original image coordinates therefore require -1.
             */
            int x = static_cast<int>(
                std::round((slice.x_ - 1.0) * scale)
            );

            int y1 = static_cast<int>(
                std::round((interval.start_.y_ - 1.0) * scale)
            );

            int y2 = static_cast<int>(
                std::round((interval.end_.y_ - 1.0) * scale)
            );

            cv::line(
                display,
                {x, y1},
                {x, y2},
                cv::Scalar(0, 0, 255),
                2
            );
        }
    }

    cv::namedWindow(
        "BCD Vertical Free Intervals",
        cv::WINDOW_NORMAL
    );

    cv::imshow(
        "BCD Vertical Free Intervals",
        display
    );

    cv::waitKey(0);
}


std::vector<BoustrophedonDecomposer::IntervalConnection> BoustrophedonDecomposer::overlapGraph(const BoustrophedonDecomposer::VerticalSlice& slice_left , const BoustrophedonDecomposer::VerticalSlice& slice_right){
    std::vector<IntervalConnection> interval_connections_;
    for(int i = 0 ; i < slice_left.free_intervals_.size() ; i++){
        auto left_free_interval = slice_left.free_intervals_.at(i);
        for (int j = 0 ; j < slice_right.free_intervals_.size() ; j++){
            auto right_free_interval = slice_right.free_intervals_.at(j);

            /* 
                overlap test of (a1,a2) and (b1,b2)
                max(a1,b1) < min(a2,b2)

                If two intervals merely touch at one endpoint:
                    A = [1, 100]
                    B = [100, 200]
                their overlap has zero length. You generally do not want to treat that as an open free-space connection for BCD.
                
            */
            if (std::max(left_free_interval.start_.y_ , right_free_interval.start_.y_) < std::min(left_free_interval.end_.y_ , right_free_interval.end_.y_)) {
                // there is an overlap
                interval_connections_.push_back({i,j});
            }

        }
    }

    return interval_connections_;


}

/*

    Conceptually, BCD now means:
    Track each free interval from left to right. As long as its connectivity stays 1→1, it belongs to the same cell. Whenever connectivity splits or merges, end the old cell(s) and start new cell(s).

    We already have everything needed:
    - VerticalSlice tells you the free intervals at each x-slab.
    - successors[L] tells you where a left interval goes.
    - predecessors[R] tells you where a right interval came from.
    Process the sweep like this:
    1. At the first slice, every free interval starts a new BCD cell.
    2. Move to the next slice.
    3. For each connection:
    - 1 → 1: same cell continues. The right interval gets the same cell ID.
    - 1 → many: split. The old cell ends; each right branch starts a new cell.
    - many → 1: merge. All incoming cells end; the right interval starts a new cell.
    - 0 → 1: a new cell starts.
    - 1 → 0: that cell ends.
    4. Repeat through all slices.
    5. At the final slice, close any cells still active.

*/
/*
    inputs:
        left slice
        right slice
        successors
        predecessors

    job:
        assign cell IDs to right slice
        based on IDs already present in left slice

*/

void BoustrophedonDecomposer::propagateCellIDs(const std::vector<std::vector<int>>& successor , const std::vector<std::vector<int>>& predecessors, BoustrophedonDecomposer::VerticalSlice& slice_left, BoustrophedonDecomposer::VerticalSlice& slice_right , int& next_cell_id_) {
    /*
        predecessors[R].size() == 0
            START

        predecessors[R].size() > 1
            MERGE

        predecessors[R].size() == 1
            let L = its only predecessor

            successors[L].size() == 1
                CONTINUE

            successors[L].size() > 1
                SPLIT
    
    */
    /*
        About the adjacency graph:
        You only add graph edges at SPLIT and MERGE:
        CONTINUE
        Cell 4 -> Cell 4

        No graph edge.
        It is literally the same cell.

        At a split:
                    Cell 6
                    /
        Cell 4 ----
                    \
                    Cell 7

        record:
        4 -- 6
        4 -- 7

        At a merge:
        Cell 6 ----\
                    >---- Cell 8
        Cell 7 ----/

        record:
        6 -- 8
        7 -- 8

        START and END do not create adjacency by themselves.
    
    */
    slice_right.cell_ids_.clear();

    for (int i = 0 ; i < predecessors.size() ; i++) {
        const auto& current_preds = predecessors.at(i);
        if(current_preds.size() == 0) {
            //Start --> Assign new cell id
            slice_right.cell_ids_.push_back(next_cell_id_);
            next_cell_id_++;

        }
        else if (current_preds.size() > 1) {
            // Many to one --> merge --> Assign a new cell id
            slice_right.cell_ids_.push_back(next_cell_id_);
            next_cell_id_++;
        }
        else if (current_preds.size() == 1) {
            // Continue
            auto ind = current_preds.at(0);
            if (successor.at(ind).size() == 1){
                // continue --> continue using the ind cell id for this one 
                slice_right.cell_ids_.push_back(slice_left.cell_ids_.at(ind));
            }
            else {
                // SPLIT --> Assign a new cell id
                slice_right.cell_ids_.push_back(next_cell_id_);
                next_cell_id_++;
            }
        }

    }



    /*
        PHASE 2:
        Find adjacency between BCD cells.

        After Phase 1, every interval on both slices already has a cell ID.
        just compare their cell IDs.
        If:
        left cell ID == right cell ID

        then it is the same BCD cell continuing:
        CELL 4 -------- CELL 4
        No adjacency edge.
        If:
        left cell ID != right cell ID
        then two different BCD cells touch at this sweep boundary:
        CELL 4 -------- CELL 5
        Therefore:
        4 and 5 are neighbors
    */


    for(int L = 0 ; L < successor.size() ; L++) {

        auto left_cell_id_ = slice_left.cell_ids_.at(L);

        for (int R : successor.at(L) ){
            auto right_cell_id_ = slice_right.cell_ids_.at(R);
            
            if (left_cell_id_ == right_cell_id_) {
                continue;
            }

            // Different cell IDs connected across this boundary:
            // they are adjacent BCD cells.
            graph_.addUndirectedEdge(
                left_cell_id_,
                right_cell_id_
            );

        }
    }



}


cv::Scalar BoustrophedonDecomposer::colorFromCellId(int cell_id) const
{
    int b = 50 + (cell_id * 53) % 206;
    int g = 50 + (cell_id * 97) % 206;
    int r = 50 + (cell_id * 193) % 206;

    return cv::Scalar(b, g, r);
}


void BoustrophedonDecomposer::showBCDCellIDs(const cv::Mat& original_image)
{
    if (vertical_slices_.empty())
    {
        std::cout << "No vertical slices. Call BCD() first!\n";
        return;
    }

    cv::Mat display;

    if (original_image.channels() == 1)
        cv::cvtColor(original_image, display, cv::COLOR_GRAY2BGR);
    else
        display = original_image.clone();

    /*
        vertical_slices_ are based on polygon coordinates that still came
        from the padded working-grid convention.

        So for image visualization, shift by (-1,-1), same as before.
    */

    // ------------------------------------------------------------
    // Draw each interval and its cell ID
    // ------------------------------------------------------------
    for (const auto& slice : vertical_slices_)
    {
        if (slice.free_intervals_.size() != slice.cell_ids_.size())
        {
            std::cout << "Mismatch: free_intervals_ and cell_ids_ sizes differ at slice x = "
                      << slice.x_ << "\n";
            continue;
        }

        for (std::size_t i = 0; i < slice.free_intervals_.size(); ++i)
        {
            const auto& interval = slice.free_intervals_[i];
            int cell_id = slice.cell_ids_[i];

            cv::Scalar color = colorFromCellId(cell_id);

            int x  = cvRound(slice.x_ - 1.0);
            int y1 = cvRound(interval.start_.y_ - 1.0);
            int y2 = cvRound(interval.end_.y_   - 1.0);

            cv::Point p1{x, y1};
            cv::Point p2{x, y2};

            // draw interval
            cv::line(display, p1, p2, color, 2, cv::LINE_AA);

            // draw endpoints
            cv::circle(display, p1, 2, color, -1);
            cv::circle(display, p2, 2, color, -1);

            // label near midpoint
            int ym = (y1 + y2) / 2;
            std::string label = "C" + std::to_string(cell_id);

            cv::putText(display,
                        label,
                        cv::Point{x + 4, ym},
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.4,
                        color,
                        1,
                        cv::LINE_AA);
        }
    }

    // ------------------------------------------------------------
    // Optional: draw links between neighboring slices when
    // the SAME cell ID continues from left to right
    // ------------------------------------------------------------
    for (std::size_t s = 0; s + 1 < vertical_slices_.size(); ++s)
    {
        const auto& left  = vertical_slices_[s];
        const auto& right = vertical_slices_[s + 1];

        if (left.free_intervals_.size() != left.cell_ids_.size() ||
            right.free_intervals_.size() != right.cell_ids_.size())
        {
            continue;
        }

        for (std::size_t i = 0; i < left.free_intervals_.size(); ++i)
        {
            for (std::size_t j = 0; j < right.free_intervals_.size(); ++j)
            {
                if (left.cell_ids_[i] != right.cell_ids_[j])
                    continue;

                int cell_id = left.cell_ids_[i];
                cv::Scalar color = colorFromCellId(cell_id);

                int x1 = cvRound(left.x_  - 1.0);
                int x2 = cvRound(right.x_ - 1.0);

                int y1 = cvRound((left.free_intervals_[i].start_.y_ +
                                  left.free_intervals_[i].end_.y_) / 2.0 - 1.0);

                int y2 = cvRound((right.free_intervals_[j].start_.y_ +
                                  right.free_intervals_[j].end_.y_) / 2.0 - 1.0);

                cv::line(display,
                         cv::Point{x1, y1},
                         cv::Point{x2, y2},
                         color,
                         1,
                         cv::LINE_AA);
            }
        }
    }

    cv::imshow("BCD Cell IDs", display);
    cv::waitKey(0);
}

void BoustrophedonDecomposer::showBCDCells( const cv::Mat& original_image, bool use_unified_polygon) {
    if (bcd_cells_.empty())
    {
        std::cout
            << "No BCD cells available. Run compute() first.\n";
        return;
    }


    cv::Mat display;

    if (original_image.channels() == 1)
    {
        cv::cvtColor(
            original_image,
            display,
            cv::COLOR_GRAY2BGR
        );
    }
    else
    {
        display = original_image.clone();
    }


    cv::Mat overlay = display.clone();


    for (const auto& cell : bcd_cells_)
    {
        cv::Scalar color(
            50 + (cell.id_ * 53)  % 206,
            50 + (cell.id_ * 97)  % 206,
            50 + (cell.id_ * 193) % 206
        );


        // =========================================================
        // Unified representation
        // =========================================================
        if (use_unified_polygon)
        {
            std::vector<cv::Point> cv_polygon;

            for (const auto& p : cell.polygon_.points_)
            {
                cv_polygon.push_back(
                    cv::Point{
                        cvRound(p.x_ - 1.0),
                        cvRound(p.y_ - 1.0)
                    }
                );
            }


            if (cv_polygon.size() >= 3)
            {
                std::vector<std::vector<cv::Point>> polygons{
                    cv_polygon
                };

                cv::fillPoly(
                    overlay,
                    polygons,
                    color
                );


                // Useful while validating the unified polygon
                cv::polylines(
                    overlay,
                    polygons,
                    true,
                    cv::Scalar(0, 0, 0),
                    1,
                    cv::LINE_AA
                );
            }
        }


        // =========================================================
        // Fragment representation
        // =========================================================
        else
        {
            for (const auto& fragment : cell.fragments_)
            {
                std::vector<cv::Point> cv_polygon;

                for (const auto& p : fragment.points_)
                {
                    cv_polygon.push_back(
                        cv::Point{
                            cvRound(p.x_ - 1.0),
                            cvRound(p.y_ - 1.0)
                        }
                    );
                }


                if (cv_polygon.size() >= 3)
                {
                    std::vector<std::vector<cv::Point>> polygons{
                        cv_polygon
                    };

                    cv::fillPoly(
                        overlay,
                        polygons,
                        color
                    );
                }
            }
        }
    }


    // Blend colored cells with original image
    cv::addWeighted(
        overlay,
        0.45,
        display,
        0.55,
        0.0,
        display
    );


    // =============================================================
    // Labels
    // =============================================================
    for (const auto& cell : bcd_cells_)
    {
        double sum_x = 0.0;
        double sum_y = 0.0;
        int count = 0;


        if (use_unified_polygon)
        {
            for (const auto& p : cell.polygon_.points_)
            {
                sum_x += p.x_;
                sum_y += p.y_;
                count++;
            }
        }
        else
        {
            for (const auto& fragment : cell.fragments_)
            {
                for (const auto& p : fragment.points_)
                {
                    sum_x += p.x_;
                    sum_y += p.y_;
                    count++;
                }
            }
        }


        if (count == 0)
            continue;


        int text_x =
            cvRound(sum_x / count - 1.0);

        int text_y =
            cvRound(sum_y / count - 1.0);


        std::string label =
            "C" + std::to_string(cell.id_);


        cv::putText(
            display,
            label,
            cv::Point{text_x, text_y},
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(0, 0, 0),
            2,
            cv::LINE_AA
        );

        cv::putText(
            display,
            label,
            cv::Point{text_x, text_y},
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(255, 255, 255),
            1,
            cv::LINE_AA
        );
    }


    std::string window_name =
        use_unified_polygon
            ? "BCD Cells - Unified"
            : "BCD Cells - Fragments";


    cv::imshow(window_name, display);
    cv::waitKey(0);
}

const CellGraph& BoustrophedonDecomposer::getGraph() const {
    return graph_;
}

const std::vector<BCDCell>& BoustrophedonDecomposer::getCells() const {
    return bcd_cells_;
}