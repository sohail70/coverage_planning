#pragma once

/*
    Coverage generation = how do I physically cover THIS CELL? 
    Here we work with the results of the BoustrophedonDecomposer's compute function (bcd_cells_) 
    So we have lots of cells and each cell consist of one unified maybe complex polygon_ and we put lanes in there!
*/

#include "cleaner/bcd_type.hpp"

class BoustrophedonCoverageGenerator {
    public:
        void compute(const std::vector<BCDCell>& bcd_cells_);
    private:

};
