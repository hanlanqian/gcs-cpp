#pragma once
#include <drake/math/matrix_util.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/context.h>

#include <unordered_map>
#include <thread>
namespace helpers
{
    void filterCollsionGeometry(drake::geometry::SceneGraph<double> &scene_graph, drake::systems::Context<double> *context);
    std::unordered_map<std::string, std::vector<double>> getConfigurationSeeds();
    std::unordered_map<std::string, drake::math::RigidTransform<double>> getIkSeeds();

    template <typename T>
    std::vector<T> concatenateVectors(const std::initializer_list<std::vector<T>> &vectors);
} // namespace helpers
