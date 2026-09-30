#include<iostream>
#include<vector>
#include<math.h>

enum class LineType{
    COVERAGE,
    TRANSITION,
};

// y = mx + y0 , m = Dy/Dx
struct Line{
    double x_start;
    double y_start;
    double x_end;
    double y_end;
    LineType type_;
};
struct Rectangle{
    double x0;
    double y0;
    double width_;
    double height_;
};

struct Trajectory {
    std::vector<Line> lines_;
};

struct Metrics{
    double coverage_distance = 0;
    double transition_distance = 0;
    double trajectory_distance = 0;
};

double lineLength(const Line& line_){
    return std::sqrt(std::pow(line_.x_end - line_.x_start , 2) + std::pow(line_.y_end - line_.y_start , 2));
}


Metrics calcMetrics(const Trajectory& coverage_traj_){
    Metrics metrics_;
    for (const auto& line_ : coverage_traj_.lines_){
        if (line_.type_ == LineType::COVERAGE){
            metrics_.coverage_distance += lineLength(line_);
        }
        if (line_.type_ == LineType::TRANSITION){
            metrics_.transition_distance += lineLength(line_);
        }

    }
    metrics_.trajectory_distance = metrics_.transition_distance + metrics_.coverage_distance;
    return metrics_;
}


int main(int argc , char* argv[]) {

    Rectangle rectangle{0,0,100,200};
    double spacing = 0.5; //0.5 meters spacing

    // Horizontal Lines
    int count = (rectangle.height_/spacing);
    std::cout<<"How many lines? " <<count<<"\n";
    Trajectory coverage_traj_;
    Line prev_coverage_line_;
    for (int i = 0 ; i < count ; i++){


        // Coverage lines
        // Start from left to right and then the next one should be right to left
        Line current_line_;
        if(i%2==0){
            current_line_ = Line{0 , i*spacing + spacing/2, rectangle.width_ , i*spacing + spacing/2, LineType::COVERAGE};
        }
        else{
            current_line_ = Line{rectangle.width_ , i*spacing + spacing/2, 0 , i*spacing + spacing/2 , LineType::COVERAGE};

        }
        // Transition Lines addition
        if (i>0){
            coverage_traj_.lines_.push_back(Line{prev_coverage_line_.x_end,
                                                 prev_coverage_line_.y_end,
                                                 current_line_.x_start,   
                                                 current_line_.y_start,   
                                                 LineType::TRANSITION
                                                });

        }
        coverage_traj_.lines_.push_back(current_line_);
        prev_coverage_line_ = current_line_;
    }

    auto metrics_  = calcMetrics(coverage_traj_);
    
    std::cout<<"Traj distance: "<<metrics_.trajectory_distance<<"\n"
            <<"coverage distance: "<<metrics_.coverage_distance<<"\n"
            <<"transition distance: "<<metrics_.transition_distance<<"\n";

}