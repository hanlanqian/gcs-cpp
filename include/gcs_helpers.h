#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/solvers/mathematical_program.h>
#include <unordered_map>
namespace gcs_helpers
{
    std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::EdgeId, double> extractEdgeFlows(drake::geometry::optimization::GraphOfConvexSets &gcs, drake::solvers::MathematicalProgramResult &result);
    std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge*> greedyForwardPathSearch(drake::geometry::optimization::GraphOfConvexSets &gcs, drake::solvers::MathematicalProgramResult &result, drake::geometry::optimization::GraphOfConvexSets::Vertex *source, drake::geometry::optimization::GraphOfConvexSets::Vertex *target, int max_paths = 10, int max_trials = 100, double flow_tol = 1e-5);
    std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge*> mipPathExtraction(drake::geometry::optimization::GraphOfConvexSets &gcs, drake::solvers::MathematicalProgramResult &result, drake::geometry::optimization::GraphOfConvexSets::Vertex *source, drake::geometry::optimization::GraphOfConvexSets::Vertex *target);
    std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>> outgoingEdges(drake::geometry::optimization::GraphOfConvexSets &gcs);
    std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>> incomingEdges(drake::geometry::optimization::GraphOfConvexSets &gcs);
    std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *> depthFirst(
        drake::geometry::optimization::GraphOfConvexSets::Vertex *source,
        drake::geometry::optimization::GraphOfConvexSets::Vertex *target,
        std::function<std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>(drake::geometry::optimization::GraphOfConvexSets::Vertex *, std::vector<drake::geometry::optimization::GraphOfConvexSets::Vertex *> &)> getCandidateEdgesFn,
        std::function<drake::geometry::optimization::GraphOfConvexSets::Edge *(std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *> &)> edgeSelectorFn);
} // namespace gcs_helpers
