#include "helpers.h"
#include "gcs.h"

#include <drake/systems/framework/diagram_builder.h>

#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/parsing/model_directives.h>
#include <drake/multibody/parsing/process_model_directives.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/joint.h>

#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/affine_ball.h>
#include <drake/geometry/optimization/hyperellipsoid.h>
#include <drake/geometry/optimization/iris.h>

#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/meshcat_visualizer_params.h>

#include <Eigen/Core>

int main(int argc, char const *argv[])
{

    // simulation setup
    auto meshcat = new drake::geometry::Meshcat();

    auto db = drake::systems::DiagramBuilder<double>();
    auto res = drake::multibody::AddMultibodyPlantSceneGraph(&db, 0.0);
    auto parser = drake::multibody::Parser(&res.plant);
    parser.package_map().Remove("drake_models");
    parser.package_map().Add("gcs", "/home/elaina/projects/gcs-science-robotics");
    parser.package_map().Add("drake_models", "/home/elaina/projects/models");
    const std::string directives_file = parser.package_map().GetPath("gcs") + "/models/bimanual_iiwa.yaml";
    auto directives = drake::multibody::parsing::LoadModelDirectives(directives_file);
    auto models = drake::multibody::parsing::ProcessModelDirectives(directives, &parser);
    drake::multibody::parsing::ModelInstanceInfo iiwa_1 = models[0], wsg_1 = models[1], iiwa_2 = models[2], wsg_2 = models[3], shelf = models[4], bin_1 = models[5], bin_2 = models[6], table = models[7];
    res.plant.Finalize();
    auto meshcat_params = drake::geometry::MeshcatVisualizerParams();
    meshcat_params.delete_on_initialization_event = false;
    meshcat_params.role = drake::geometry::Role::kIllustration;
    std::shared_ptr<drake::geometry::Meshcat> meshcat_ptr = std::shared_ptr<drake::geometry::Meshcat>(meshcat);
    auto meshcat_app = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(&db, res.scene_graph, meshcat_ptr, meshcat_params);

    auto diagram = db.Build();
    // set initial state
    Eigen::Vector<double, 7> q0(0.0, -0.2, 0, -1.2, 0.0, 1.6, 0.0);
    int index = 0;
    for (drake::multibody::JointIndex joint_index : res.plant.GetJointIndices(iiwa_1.model_instance))
    {
        auto joint = &res.plant.get_mutable_joint(joint_index);
        drake::log()->info(joint->type_name());
        if (joint->type_name() == "revolute")
        {
            joint->set_default_positions(Eigen::Matrix<double, 1, 1>{q0[index]});
            index++;
        }
    }
    index = 0;
    for (drake::multibody::JointIndex joint_index : res.plant.GetJointIndices(iiwa_2.model_instance))
    {
        auto joint = &res.plant.get_mutable_joint(joint_index);
        drake::log()->info(joint->type_name());
        if (joint->type_name() == "revolute")
        {
            joint->set_default_positions(Eigen::Matrix<double, 1, 1>{q0[index]});
            index++;
        }
    }
    // publish initial context
    auto context = diagram->CreateDefaultContext();
    auto plant_context = &res.plant.GetMyMutableContextFromRoot(context.get());
    auto sg_context = &res.scene_graph.GetMyMutableContextFromRoot(context.get());
    diagram->ForcedPublish(*context);

    // IRIS Region Generations
    auto iris_options = drake::geometry::optimization::IrisOptions();
    iris_options.require_sample_point_is_contained = true;
    iris_options.iteration_limit = 5;
    iris_options.termination_threshold = -1;
    iris_options.relative_termination_threshold = 0.02;
    iris_options.num_collision_infeasible_samples = 1;

    helpers::filterCollsionGeometry(res.scene_graph, sg_context);
    
    

    drake::log()->info("example ended.");
    while (1)
    {
        sleep(1);
        drake::log()->info("looping.");
    }
    return 0;
}