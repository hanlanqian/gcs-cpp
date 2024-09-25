#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/solvers/mathematical_program.h>
#include <unordered_map>
namespace gcs_helpers
{
    void greedyForwardPathSearch(drake::geometry::optimization::GraphOfConvexSets &gcs, drake::solvers::MathematicalProgramResult &result, drake::geometry::optimization::GraphOfConvexSets::Vertex *source, drake::geometry::optimization::GraphOfConvexSets::Vertex *target, int max_paths = 10, int max_trials = 100, double flow_tol = 1e-5);
    std::unordered_map<drake::geometry::optimization::GraphOfConvexSets::VertexId, std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge*>> outgoingEdges(drake::geometry::optimization::GraphOfConvexSets &gcs);
    
} // namespace gcs_helpers
