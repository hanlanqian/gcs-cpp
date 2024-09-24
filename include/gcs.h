#pragma once
#include <drake/math/matrix_util.h>
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/solvers/mosek_solver.h>
#include <drake/solvers/common_solver_option.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solver_options.h>

class BaseGCS
{
private:
    int dimension;
    std::vector<drake::geometry::optimization::HPolyhedron> regions;
    std::vector<std::string> names;
    drake::geometry::optimization::GraphOfConvexSets gcs;
    drake::geometry::optimization::GcsGraphvizOptions options;

public:
    BaseGCS();
    BaseGCS(const std::vector<drake::geometry::optimization::HPolyhedron> &regions);
    ~BaseGCS();
};

class LinearGCS : public BaseGCS
{
private:
    static std::vector<std::pair<int, int>> empty;
public:
    LinearGCS(/* args */);
    LinearGCS(const std::vector<drake::geometry::optimization::HPolyhedron> regions, const std::vector<std::pair<int, int>> &edges = empty);
    ~LinearGCS();
};
