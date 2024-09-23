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
    BaseGCS(std::vector<drake::geometry::optimization::HPolyhedron> &regions);
    ~BaseGCS();
};
