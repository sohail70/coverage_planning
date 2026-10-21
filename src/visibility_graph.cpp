#include "cleaner/visibility_graph.hpp"
#include "cleaner/geometry.hpp"
#include <cctype>

// Assuming Polygon is hole free! basically i mean its a polygon for free space not the holes (obstacle space) --> so we use this function after we are done with the BCD

WeightedGraph VisibilityGraph::compute(const Polygon<double>& polygon) {
    WeightedGraph cg_;

    for (size_t i = 0; i < polygon.points_.size(); ++i) {
        cg_.addNode(i);
    }
    
    for (size_t i = 0 ; i < polygon.points_.size() ; i++) {
        for(size_t j = i+1 ; j < polygon.points_.size() ; j++) {
            Point<double> A = polygon.points_.at(i);
            Point<double> B = polygon.points_.at(j);
            if (isVisible(A, B, polygon)) {
                double dist = distance(A, B);
                // Use the polygon vertex index as the node ID for the visibility graph.
                cg_.addUndirectedEdge(i, j, dist);
            }
        }
    }

    return cg_;
}

bool VisibilityGraph::isVisible(const Point<double>& A , const Point<double>& B, const Polygon<double>& polygon) {
    return isSegmentInsidePolygon(Segment<double>{A,B}, polygon);
}







const WeightedGraph& VisibilityGraphCache::getOrBuild( cellID id, const Polygon<double>& polygon) {
    auto it = graphs_.find(id);
    if(it != graphs_.end()) {
        return it->second;
    }
    VisibilityGraph vg_;
    auto [inserted_it_, is_successful] = graphs_.try_emplace(id, vg_.compute(polygon));
    return inserted_it_->second;
}