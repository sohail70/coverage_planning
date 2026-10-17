#pragma once

/*
    Dependency Structure:

                 cell_types.hpp
                 using cellID=int
                    /   |   \
                   /    |    \
                  v     v     v
             BCDCell  CellGraph  CellTraversal
                |        |          |
                |        |          v
                |        |         DFS
                |
                v
       CellCoverageCandidate

*/


using cellID = int;