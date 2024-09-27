#include "gcs_helpers.h"

std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::EdgeId, double> gcs_helpers::extractEdgeFlows(drake::geometry::optimization::GraphOfConvexSets &gcs, drake::solvers::MathematicalProgramResult &result)
{
    std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::EdgeId, double> flows;
    for (auto &e : gcs.Edges())
    {
        flows[e->id()] = result.GetSolution(e->phi());
    }
    return flows;
}

std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge*> gcs_helpers::greedyForwardPathSearch(drake::geometry::optimization::GraphOfConvexSets &gcs, drake::solvers::MathematicalProgramResult &result, drake::geometry::optimization::GraphOfConvexSets::Vertex *source, drake::geometry::optimization::GraphOfConvexSets::Vertex *target, int max_paths, int max_trials, double flow_tol)
{
    auto outgoing_edges = outgoingEdges(gcs);
    auto flows = extractEdgeFlows(gcs, result);
    auto getCandidateEdgesFn = [&outgoing_edges, &flows, &flow_tol](drake::geometry::optimization::GraphOfConvexSets::Vertex *current_vertex, std::vector<drake::geometry::optimization::GraphOfConvexSets::Vertex *> &visited_vertices)
    {
        std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *> candidates;
        for (const auto &edge : outgoing_edges[current_vertex->id()])
        {
            if (std::find(visited_vertices.begin(), visited_vertices.end(), &edge->v()) == visited_vertices.end() && flows[edge->id()] > flow_tol)
            {
                candidates.push_back(edge);
            }
        }
        return candidates;
    };
    auto edgeSelectorFn = [&flows](std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *> &candidate_edges)
    {
        drake::geometry::optimization::GraphOfConvexSets::Edge *max_flow_edge(nullptr);
        for (auto &e : candidate_edges)
        {
            if (!max_flow_edge || flows[max_flow_edge->id()] < flows[e->id()])
            {
                max_flow_edge = e;
            }
        }
        return max_flow_edge;
    };
    return depthFirst(source, target, getCandidateEdgesFn, edgeSelectorFn);
}

std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge*> gcs_helpers::mipPathExtraction(drake::geometry::optimization::GraphOfConvexSets &gcs, drake::solvers::MathematicalProgramResult &result, drake::geometry::optimization::GraphOfConvexSets::Vertex *source, drake::geometry::optimization::GraphOfConvexSets::Vertex *target)
{
    return greedyForwardPathSearch(gcs, result, source, target);
}

std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>> gcs_helpers::outgoingEdges(drake::geometry::optimization::GraphOfConvexSets &gcs)
{
    std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>> edges;
    for (auto &v : gcs.Vertices())
    {
        edges[v->id()] = {};
    }
    for (auto &e : gcs.Edges())
    {
        edges[e->u().id()].push_back(e);
    }
    return edges;
}

std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>> gcs_helpers::incomingEdges(drake::geometry::optimization::GraphOfConvexSets &gcs)
{
    std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>> edges;
    for (auto &v : gcs.Vertices())
    {
        edges[v->id()] = {};
    }
    for (auto &e : gcs.Edges())
    {
        edges[e->v().id()].push_back(e);
    }
    return edges;
}

std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *> gcs_helpers::depthFirst(drake::geometry::optimization::GraphOfConvexSets::Vertex *source, drake::geometry::optimization::GraphOfConvexSets::Vertex *target, std::function<std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>(drake::geometry::optimization::GraphOfConvexSets::Vertex *, std::vector<drake::geometry::optimization::GraphOfConvexSets::Vertex *> &)> getCandidateEdgesFn, std::function<drake::geometry::optimization::GraphOfConvexSets::Edge *(std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *> &)> edgeSelectorFn)
{
    std::vector<drake::geometry::optimization::GraphOfConvexSets::Vertex *> visited_vertices;
    visited_vertices.push_back(source);
    std::vector<drake::geometry::optimization::GraphOfConvexSets::Vertex *> path_vertices = {source};
    std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *> path_edges;

    while (path_vertices.back() != target)
    {
        auto candidate_edges = getCandidateEdgesFn(path_vertices.back(), visited_vertices);
        if (candidate_edges.empty())
        {
            path_vertices.pop_back();
            if (!path_edges.empty())
            {
                path_edges.pop_back();
            }
        }
        else
        {
            auto selected = edgeSelectorFn(candidate_edges);
            visited_vertices.push_back(&selected->v());
            path_vertices.push_back(&selected->v());
            path_edges.push_back(selected);
        }
    }
    return path_edges;
}
