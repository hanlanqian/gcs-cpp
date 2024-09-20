#pragma once
#include <drake/math/matrix_util.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/context.h>

#include <unordered_map>
#include <thread>
namespace helpers
{
    void filterCollsionGeometry(drake::geometry::SceneGraph<double> &scene_graph, drake::systems::Context<double> *context);
    std::unordered_map<std::string, Eigen::Matrix<double, 2, 7>> getConfigurationSeed();
} // namespace helpers
