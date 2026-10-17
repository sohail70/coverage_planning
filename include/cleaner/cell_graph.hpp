#pragma once
#include "cleaner/cell_types.hpp"
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <iostream>

/*
    After decomposition we need to create this type because traversal class shouldnt care 
    about the BCD or any other decomposition mthod and deal
    with abstract type of CellGraph
    so each type of decomposition needs to return CellGraph via a method like getGraph()
*/

struct CellEdge {
    cellID to_;
    double weight_ = 1.0;
};


class CellGraph {
    public:

        void addCell(cellID id) {
            adjacency_.try_emplace(id);
        } 
        void addUndirectedEdge(cellID a , cellID b, double weight = 1.0) {
            if (a==b) return;
            addCell(a);
            addCell(b);
            auto& a_neighbors_ = adjacency_.at(a);
            auto& b_neighbors_ = adjacency_.at(b);

            bool a_has_b = std::any_of(a_neighbors_.begin(), a_neighbors_.end(), [&](const CellEdge& edge){
                return edge.to_ == b;
            });

            bool b_has_a = std::any_of(b_neighbors_.begin(), b_neighbors_.end(), [&](const CellEdge& edge){
                return edge.to_ == a;
            } );

            if (!a_has_b) {
                a_neighbors_.push_back({b,weight});
            }
            if (!b_has_a) {
                b_neighbors_.push_back({a, weight});
            }

        }

        const std::vector<CellEdge>& neighbors(cellID id) const {
            return adjacency_.at(id);
        }

        void clear() {
            adjacency_.clear();
        }

        void print(std::ostream& out) const
        {
            // unordered_map does not preserve order,
            // so sort the IDs to make debugging easier.
            std::vector<cellID> ids;

            for (const auto& [id, edges] : adjacency_)
            {
                ids.push_back(id);
            }

            std::sort(ids.begin(), ids.end());


            std::size_t directed_edge_count = 0;

            for (cellID id : ids)
            {
                out << "CELL " << id << " neighbors: ";

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

    private:
        std::unordered_map<cellID, std::vector<CellEdge>> adjacency_; // <1,2> --> from 1 to 2 --> so a cell(node) can connect to many other cells with an edge with a weight
};