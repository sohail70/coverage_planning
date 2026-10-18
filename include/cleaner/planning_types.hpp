#pragma once

/*
    Dependency Structure:

               graph_types.hpp
                   nodeID
                  /     \
                 v       v
        WeightedGraph   planning_types.hpp
                            |
                         cellID
                            |
                          BCDCell

*/

/*
   graph_types.hpp
      using nodeID = int;

   planning_types.hpp
      #include "graph_types.hpp"
      using cellID = nodeID;

*/

using nodeID = int;
using cellID = nodeID; // Concept used for BCD cells