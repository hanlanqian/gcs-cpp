#pragma once
#include <drake/math/matrix_util.h>
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/solvers/mosek_solver.h>
#include <drake/solvers/common_solver_option.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solver_options.h>
#include <drake/solvers/binding.h>
#include <drake/solvers/mosek_solver.h>

#include <variant>
class BaseGCS
{
private:
    int _dimension;

public:
    std::vector<drake::geometry::optimization::HPolyhedron> regions;
    std::vector<std::string> names;
    drake::geometry::optimization::GraphOfConvexSets gcs;
    drake::geometry::optimization::GraphOfConvexSetsOptions options;
    drake::geometry::optimization::GraphOfConvexSets::Vertex *source, *target;
    int dimension();
    BaseGCS(const std::vector<drake::geometry::optimization::HPolyhedron> &regions);
    ~BaseGCS();
    std::vector<std::pair<int, int>> findEdgesViaFullDimensionOverlaps();
    std::vector<std::pair<int, int>> findEdgesViaOverlaps();
    std::array<std::vector<int>, 2> findStartGoalEdges(drake::VectorX<double> &start, drake::VectorX<double> &goal);
    std::array<std::vector<drake::geometry::optimization::GraphOfConvexSets::Edge *>, 2> addSourceTarget(drake::VectorX<double> &source, drake::VectorX<double> &target, std::array<std::vector<int>, 2> edges = {});
    void solveGCS(bool rounding=false, bool preprocessing=false, bool verbose=false);
};  

class LinearGCS : public BaseGCS
{
private:
    drake::solvers::L2NormCost edge_costs;

public:
    LinearGCS(const std::vector<drake::geometry::optimization::HPolyhedron> regions, std::vector<std::pair<int, int>> edges = {}, drake::VectorX<double> path_weights = {}, bool full_dim_overlap = false);
    void addSourceTarget(drake::VectorX<double> &source, drake::VectorX<double> &target, std::array<std::vector<int>, 2> edges = {});
    ~LinearGCS();
};
