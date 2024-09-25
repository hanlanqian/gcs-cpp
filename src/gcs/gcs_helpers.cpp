#include "gcs_helpers.h"

std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge*>> gcs_helpers::outgoingEdges(drake::geometry::optimization::GraphOfConvexSets &gcs)
{
    std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge*>> edges;
    for(auto &v : gcs.Vertices()){
        edges[v->id()] = {};  
    }
    for (auto &e : gcs.Edges()){
        edges[e->u().id()].push_back(e);
    }
    return edges;
}