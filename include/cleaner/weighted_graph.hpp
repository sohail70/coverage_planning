#pragma once
#include "cleaner/planning_types.hpp"
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <iostream>


/*
                WeightedGraph
                    nodeID
                    adjacency
                    weighted edges

                ↑                  ↑
                |                  |
            WeightedGraph          VisibilityGraph
            BCD-cell meaning   geometric-point meaning


            current WeightedGraph means:
            node = BCD cell
            edge = two BCD cells are adjacent
            weight = some cell-to-cell/topological cost

            A visibility graph means:
            node = geometric point / polygon vertex
            edge = two points can see each other
            weight = Euclidean distance



*/


/*
    After decomposition we need to create this type because traversal class shouldnt care 
    about the BCD or any other decomposition mthod and deal
    with abstract type of WeightedGraph
    so each type of decomposition needs to return WeightedGraph via a method like getGraph()
*/

struct GraphEdge {
    nodeID to_;
    double weight_ = 1.0;
};


class WeightedGraph {
    public:

        void addNode(nodeID id) {
            adjacency_.try_emplace(id);
        } 
        void addUndirectedEdge(nodeID a , nodeID b, double weight = 1.0) {
            if (a==b) return;
            addNode(a);
            addNode(b);
            auto& a_neighbors_ = adjacency_.at(a);
            auto& b_neighbors_ = adjacency_.at(b);

            bool a_has_b = std::any_of(a_neighbors_.begin(), a_neighbors_.end(), [&](const GraphEdge& edge){
                return edge.to_ == b;
            });

            bool b_has_a = std::any_of(b_neighbors_.begin(), b_neighbors_.end(), [&](const GraphEdge& edge){
                return edge.to_ == a;
            } );

            if (!a_has_b) {
                a_neighbors_.push_back({b,weight});
            }
            if (!b_has_a) {
                b_neighbors_.push_back({a, weight});
            }

        }

        const std::vector<GraphEdge>& neighbors(nodeID id) const {
            return adjacency_.at(id);
        }

        void clear() {
            adjacency_.clear();
        }

        void print(std::ostream& out) const
        {
            // unordered_map does not preserve order,
            // so sort the IDs to make debugging easier.
            std::vector<nodeID> ids;

            for (const auto& [id, edges] : adjacency_)
            {
                ids.push_back(id);
            }

            std::sort(ids.begin(), ids.end());


            std::size_t directed_edge_count = 0;

            for (nodeID id : ids)
            {
                out << "NODE " << id << " neighbors: ";

                for (const auto& edge : adjacency_.at(id))
                {
                    out << edge.to_
                        << "(w=" << edge.weight_ << ") ";

                    directed_edge_count++;
                }

                out << "\n";
            }


            out << "\nNodes: "
                << adjacency_.size()
                << "\n";

            // Every undirected edge is stored twice:
            // A -> B
            // B -> A
            out << "Undirected edges: "
                << directed_edge_count / 2
                << "\n";
        }

        size_t nodeCount() {
            return adjacency_.size();
        }
      

    private:
        std::unordered_map<nodeID, std::vector<GraphEdge>> adjacency_; // <1,2> --> from 1 to 2 --> so a node can connect to many other cells with an edge with a weight
};