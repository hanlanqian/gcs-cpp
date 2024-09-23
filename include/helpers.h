#pragma once
#include <drake/math/matrix_util.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/optimization/iris.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <unordered_map>
#include <thread>
namespace helpers
{
    void filterCollsionGeometry(drake::geometry::SceneGraph<double> &scene_graph, drake::systems::Context<double> *context);
    std::unordered_map<std::string, drake::VectorX<double>> getConfigurationSeeds();
    std::unordered_map<std::string, drake::math::RigidTransform<double>> getIkSeeds();
    drake::geometry::optimization::HPolyhedron calcRegion(int i, Eigen::Ref<const drake::VectorX<double>> &seed, drake::multibody::MultibodyPlant<double> &plant, drake::systems::Context<double> *context);
    std::vector<drake::geometry::optimization::HPolyhedron> generateRegions(std::unordered_map<std::string, drake::VectorX<double>> &seeds_points, drake::multibody::MultibodyPlant<double> &plant, drake::systems::Context<double> *context);

    template <typename T>
    std::vector<T> concatenateVectors(const std::initializer_list<std::vector<T>> &vectors);
} // namespace helpers
